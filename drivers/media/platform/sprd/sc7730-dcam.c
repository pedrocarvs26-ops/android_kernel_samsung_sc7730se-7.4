// SPDX-License-Identifier: GPL-2.0
/*
 * Camera capture of the Spreadtrum SC7730/SC8830 ("shark"): the Synopsys
 * DesignWare CSI-2 host (v1.03) with its two D-PHYs and the DCAM capture
 * engine, used here as a plain receiver for sensors with their own ISP.
 *
 * DCAM path 1 writes the captured YUV422 as a Y plane plus an interleaved
 * CbCr plane, 4:2:2 (NV16) with the scaler bypassed or 4:2:0 (NV12) through
 * the scaler at 1:1, which halves the chroma lines (with the scaler
 * bypassed the YUV420 mode keeps every chroma line and places them wrong,
 * which is why the vendor bypasses only for YUV422). It has no line pitch,
 * so lines are packed and the CbCr plane starts at width * height. Frame
 * addresses are shadowed: written at the path's start of frame they latch
 * at the next one (auto copy), as in the vendor driver. A scratch buffer
 * takes frames for which user space has queued nothing.
 *
 * Path 1 has a 2048 pixel line buffer, path 2 one of 4096; frames wider
 * than 2048 (the 5 MP stills of the rear camera) go through path 2, with
 * the scaler bypassed as well (module parameter path_sel to override).
 * The vendor never ran path 2 in bypass: it always trimmed one line and
 * scaled it back (path2_crop to trim lines without scaling). The number
 * of frames to drop after the start comes from the sensor.
 *
 * Port 0 is D-PHY A (2 lanes, the rear camera), port 1 D-PHY C (1 lane,
 * the front camera); only one sensor streams at a time, chosen through
 * the media links. The sensor master clock (MCLK, 24 MHz) lives in the MM
 * power domain and is provided here.
 *
 * Registers and sequences: vendor drivers/media/sprd_dcam and csi2,
 * gtelwifi vendor-ref/research/camera-m2-report.md.
 */

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

/* DCAM */
#define DCAM_CFG		0x00
#define  CFG_CSI2_IN		BIT(9)
#define DCAM_CONTROL		0x04
#define  CTL_CAP_FORCE_COPY	BIT(0)
#define  CTL_CAP_EN		BIT(2)
/* path 1 at 0x10, path 2 at 0x24 */
#define PATH_CFG		0x00
#define  P_OUT_FMT		GENMASK(7, 6)	/* 0 YUV422, 1 YUV420, 2-plane */
#define  P_UV_TAP		GENMASK(15, 11)
#define  P_Y_TAP		GENMASK(19, 16)
#define  P_SCALER_BYPASS	BIT(20)
#define PATH_SRC_SIZE		0x04
#define PATH_DST_SIZE		0x08
#define PATH_TRIM_START		0x0c
#define PATH_TRIM_SIZE		0x10
#define INT_STS			0x3c
#define INT_MASK		0x40
#define INT_CLR			0x44
#define  INT_SN_LINE_ERR	BIT(10)
#define  INT_SN_FRAME_ERR	BIT(11)
#define  INT_MIPI_OV		BIT(14)
#define  INT_ALL		GENMASK(20, 0)
#define ENDIAN_SEL		0x6c
#define  ENDIAN_AXI		(BIT(18) | BIT(19))
#define AHBM_STS		0x70
#define  AHBM_BUSY		BIT(0)
#define  AHBM_WR_ARBIT		BIT(8)
#define CAP_MIPI_CTRL		0x128
#define  CAP_MULTI_FRAME	BIT(6)
#define  CAP_YUV_ORDER		GENMASK(8, 7)	/* 2 = UYVY */
#define CAP_MIPI_FRM_CTRL	0x12c
#define  CAP_PRE_SKIP		GENMASK(3, 0)
#define  CAP_CLR_COUNT		BIT(22)
#define CAP_MIPI_START		0x130
#define CAP_MIPI_END		0x134
#define CAP_MIPI_IMG_DECI	0x138
/* scaler coefficient RAM: path 1 at 0x400, path 2 at 0x1400 */
#define SC_COEF_H		0x000
#define SC_COEF_V		0x0f0
#define SC_COEF_VC		0x4f0

/* CSI-2 host */
#define CSI_VERSION		0x00
#define  CSI_VERSION_103	0x3130332a
#define CSI_N_LANES		0x04
#define CSI_PHY_SHUTDOWNZ	0x08
#define CSI_DPHY_RSTZ		0x0c
#define CSI_RESETN		0x10
#define CSI_PHY_STATE		0x14
#define CSI_ERR1		0x20
#define CSI_ERR2		0x24
#define CSI_MASK1		0x28
#define CSI_MASK2		0x2c
#define CSI_TST_CTRL0		0x30
#define  TST_CLR		BIT(0)
#define  TST_CLK		BIT(1)
#define CSI_TST_CTRL1		0x34
#define  TST_EN			BIT(16)

/* MM AHB */
#define MM_AHB_EB		0x00
#define  EB_DCAM		BIT(0)
#define  EB_CSI			BIT(4)
#define  EB_MM_CKG		BIT(6)
#define MM_AHB_RST		0x04
#define  RST_DCAM		BIT(0)
#define  RST_CCIR		BIT(1)
#define  RST_CSI		BIT(5)
#define MM_GEN_CKG		0x08
#define  CKG_CPHY_CFG		BIT(0)
#define  CKG_MIPI_CSI		BIT(1)
#define  CKG_SENSOR		BIT(2)
#define  CKG_DCAM_AXI		BIT(3)
#define  CKG_MM_AXI		(BIT(7) | BIT(8))
#define MM_CSI2_CTRL		0x0c
#define  CSI2_PHY_SEL		GENMASK(5, 0)
#define  CSI2_PHY_A		0x0c
#define  CSI2_PHY_C		0x20

/* MM clocks */
#define MM_CLK_AHB		0x20	/* 3 = 153.6 MHz */
#define MM_CLK_SENSOR		0x24	/* source + divider */
#define  SENSOR_48M_DIV2	0x101
#define  SENSOR_MASK		0x703
#define MM_CLK_DCAM		0x2c	/* 2 = 256 MHz; 3 hangs the bus */

/* AON APB PWR_CTRL: D-PHY A power-down, shared with the USB PHY */
#define AON_PWR_CTRL		0x24
#define  PWR_CSI0_PHY_PD	BIT(10)

#define DCAM_MCLK_HZ		24000000
#define DCAM_PATH1_MAX_WIDTH	2048	/* line buffer; path2: 4096 */
#define DCAM_MAX_WIDTH		4092	/* capture window */
#define DCAM_MAX_HEIGHT		4092

/* the two scaling paths, from the vendor sc8830g/dcam_drv.c */
struct dcam_path {
	unsigned int id;
	u32 regs;		/* PATH_CFG .. PATH_TRIM_SIZE */
	u32 addr_y, addr_uv;	/* FRM_ADDR1/2, FRM_ADDR4/5 */
	u32 cfg_en, cfg_src;	/* DCAM_CFG */
	u32 force_copy, auto_copy, coef_force;	/* DCAM_CONTROL */
	u32 endian_little;	/* ENDIAN_SEL Y and UV fields */
	u32 ahbm_stop, ahbm_busy;
	u32 int_done, int_ov, int_sof;
	u32 rst;		/* MM_AHB_RST */
	u32 coef;		/* scaler coefficient RAM */
};

static const struct dcam_path dcam_paths[] = {
	{
		.id = 1, .regs = 0x10, .addr_y = 0x50, .addr_uv = 0x54,
		.cfg_en = BIT(1), .cfg_src = GENMASK(12, 11),
		.force_copy = BIT(10), .auto_copy = BIT(11),
		.coef_force = BIT(14),
		.endian_little = BIT(6) | BIT(8),
		.ahbm_stop = BIT(4), .ahbm_busy = BIT(17),
		.int_done = BIT(6), .int_ov = BIT(7), .int_sof = BIT(19),
		.rst = BIT(8), .coef = 0x400,
	}, {
		.id = 2, .regs = 0x24, .addr_y = 0x5c, .addr_uv = 0x60,
		.cfg_en = BIT(2), .cfg_src = GENMASK(14, 13),
		.force_copy = BIT(12), .auto_copy = BIT(13),
		.coef_force = BIT(16),
		.endian_little = BIT(10) | BIT(12),
		.ahbm_stop = BIT(5), .ahbm_busy = BIT(18),
		.int_done = BIT(8), .int_ov = BIT(9), .int_sof = BIT(20),
		.rst = BIT(9), .coef = 0x1400,
	},
};

static int path_sel;
module_param(path_sel, int, 0644);
MODULE_PARM_DESC(path_sel, "0: path 1 up to 2048 pixels wide, path 2 above; 1 or 2: always that path");

static unsigned int path2_crop;
module_param(path2_crop, uint, 0644);
MODULE_PARM_DESC(path2_crop, "Lines trimmed off the bottom on path 2, scaler still bypassed (even)");

enum { DCAM_PORT_PHY_A, DCAM_PORT_PHY_C, DCAM_NUM_PORTS };

struct dcam_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct dcam_sensor {
	struct v4l2_subdev *sd;
	unsigned int lanes;
	unsigned int port;
};

struct dcam {
	struct device *dev;
	void __iomem *base;
	void __iomem *csi;
	void __iomem *mmahb;
	void __iomem *mmclk;
	void __iomem *aon;
	int irq;

	struct clk_hw mclk;
	struct media_device mdev;
	struct v4l2_device v4l2;
	struct v4l2_async_notifier notifier;
	struct dcam_sensor sensors[DCAM_NUM_PORTS];
	struct dcam_sensor *active;	/* streaming sensor */
	const struct dcam_path *path;	/* in use while streaming */
	u32 int_errors;
	u32 skip;			/* frames dropped after the start */

	struct video_device vdev;
	struct media_pad pad;
	struct media_pipeline pipe;
	struct vb2_queue queue;
	struct mutex lock;		/* vdev and queue */
	struct v4l2_pix_format fmt;

	spinlock_t qlock;		/* everything below */
	struct list_head queued;
	struct dcam_buf *cur;		/* being written, NULL = scratch */
	struct dcam_buf *next;		/* latches at the next SOF */
	bool next_valid;
	void *scratch;
	dma_addr_t scratch_dma;
	size_t scratch_size;
	u32 sequence;
	u32 errors;
};

struct dcam_asc {
	struct v4l2_async_connection asc;
	unsigned int port;
	unsigned int lanes;
};

static inline void dcam_rmw(void __iomem *reg, u32 clr, u32 set)
{
	writel((readl(reg) & ~clr) | set, reg);
}

/* --- MM block clocks and the sensor clock ---------------------------- */

static int dcam_runtime_resume(struct device *dev)
{
	struct dcam *d = dev_get_drvdata(dev);

	/* the MM domain (genpd) is on; its clock muxes reset with it */
	dcam_rmw(d->mmahb + MM_AHB_EB, 0, EB_MM_CKG);
	dcam_rmw(d->mmahb + MM_GEN_CKG, 0, CKG_MM_AXI);
	writel(3, d->mmclk + MM_CLK_AHB);
	dcam_rmw(d->mmahb + MM_AHB_EB, 0, EB_DCAM | EB_CSI);
	dcam_rmw(d->mmahb + MM_GEN_CKG, 0,
		 CKG_CPHY_CFG | CKG_MIPI_CSI | CKG_DCAM_AXI);
	dcam_rmw(d->mmclk + MM_CLK_DCAM, 0x3, 2);
	dcam_rmw(d->mmclk + MM_CLK_SENSOR, SENSOR_MASK, SENSOR_48M_DIV2);
	return 0;
}

static int dcam_runtime_suspend(struct device *dev)
{
	struct dcam *d = dev_get_drvdata(dev);

	dcam_rmw(d->mmahb + MM_GEN_CKG,
		 CKG_CPHY_CFG | CKG_MIPI_CSI | CKG_SENSOR | CKG_DCAM_AXI, 0);
	dcam_rmw(d->mmahb + MM_AHB_EB, EB_DCAM | EB_CSI, 0);
	return 0;
}

#define to_dcam_mclk(hw) container_of(hw, struct dcam, mclk)

static int dcam_mclk_prepare(struct clk_hw *hw)
{
	return pm_runtime_resume_and_get(to_dcam_mclk(hw)->dev);
}

static void dcam_mclk_unprepare(struct clk_hw *hw)
{
	pm_runtime_put(to_dcam_mclk(hw)->dev);
}

static int dcam_mclk_enable(struct clk_hw *hw)
{
	struct dcam *d = to_dcam_mclk(hw);

	dcam_rmw(d->mmclk + MM_CLK_SENSOR, SENSOR_MASK, SENSOR_48M_DIV2);
	dcam_rmw(d->mmahb + MM_GEN_CKG, 0, CKG_SENSOR);
	return 0;
}

static void dcam_mclk_disable(struct clk_hw *hw)
{
	dcam_rmw(to_dcam_mclk(hw)->mmahb + MM_GEN_CKG, CKG_SENSOR, 0);
}

static unsigned long dcam_mclk_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	return DCAM_MCLK_HZ;
}

static int dcam_mclk_determine_rate(struct clk_hw *hw,
				    struct clk_rate_request *req)
{
	req->rate = DCAM_MCLK_HZ;
	return 0;
}

static int dcam_mclk_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	return rate == DCAM_MCLK_HZ ? 0 : -EINVAL;
}

static const struct clk_ops dcam_mclk_ops = {
	.prepare = dcam_mclk_prepare,
	.unprepare = dcam_mclk_unprepare,
	.enable = dcam_mclk_enable,
	.disable = dcam_mclk_disable,
	.recalc_rate = dcam_mclk_recalc_rate,
	.determine_rate = dcam_mclk_determine_rate,
	.set_rate = dcam_mclk_set_rate,
};

/* --- CSI-2 host and D-PHY -------------------------------------------- */

/* vendor csi_drv.c table, lane rate in Mbit/s */
static const struct {
	unsigned int min, max;
	u8 hsfreqrange, settle;
} dcam_dphy_ranges[] = {
	{ 180, 200, 0x22, 9 },
	{ 270, 300, 0x14, 13 },
	{ 330, 360, 0x15, 17 },
	{ 360, 400, 0x25, 17 },
};

static void dcam_dphy_write(struct dcam *d, u8 code, u8 data)
{
	writel(0, d->csi + CSI_TST_CTRL1);
	dcam_rmw(d->csi + CSI_TST_CTRL0, 0, TST_CLK);
	udelay(1);
	writel(code | TST_EN, d->csi + CSI_TST_CTRL1);
	udelay(1);
	dcam_rmw(d->csi + CSI_TST_CTRL0, TST_CLK, 0);	/* latch code */
	udelay(1);
	writel(code, d->csi + CSI_TST_CTRL1);
	udelay(1);
	writel(data, d->csi + CSI_TST_CTRL1);
	udelay(1);
	dcam_rmw(d->csi + CSI_TST_CTRL0, 0, TST_CLK);	/* latch data */
	udelay(1);
}

static int dcam_csi_start(struct dcam *d, struct dcam_sensor *s, s64 link_freq)
{
	unsigned int mbps = div_u64(link_freq * 2, 1000000), i;
	u32 version;

	for (i = 0; i < ARRAY_SIZE(dcam_dphy_ranges); i++)
		if (mbps >= dcam_dphy_ranges[i].min &&
		    mbps < dcam_dphy_ranges[i].max)
			break;
	if (i == ARRAY_SIZE(dcam_dphy_ranges)) {
		dev_err(d->dev, "no D-PHY settings for %u Mbit/s\n", mbps);
		return -EINVAL;
	}

	dcam_rmw(d->mmahb + MM_AHB_RST, 0, RST_CSI);
	udelay(1);
	dcam_rmw(d->mmahb + MM_AHB_RST, RST_CSI, 0);
	version = readl(d->csi + CSI_VERSION);
	if (version != CSI_VERSION_103) {
		dev_err(d->dev, "unexpected CSI-2 host version %#x\n", version);
		return -ENODEV;
	}

	if (s->port == DCAM_PORT_PHY_A) {
		dcam_rmw(d->aon + AON_PWR_CTRL, PWR_CSI0_PHY_PD, 0);
		dcam_rmw(d->mmahb + MM_CSI2_CTRL, CSI2_PHY_SEL, CSI2_PHY_A);
	} else {
		dcam_rmw(d->mmahb + MM_CSI2_CTRL, CSI2_PHY_SEL, CSI2_PHY_C);
	}

	writel(0, d->csi + CSI_PHY_SHUTDOWNZ);
	writel(0, d->csi + CSI_DPHY_RSTZ);
	writel(TST_CLR, d->csi + CSI_TST_CTRL0);
	udelay(1);
	writel(0, d->csi + CSI_TST_CTRL0);
	udelay(1);
	dcam_dphy_write(d, 0x34, 0x14);
	dcam_dphy_write(d, 0x44, dcam_dphy_ranges[i].hsfreqrange << 1);
	dcam_dphy_write(d, 0x75, 0x80 | dcam_dphy_ranges[i].settle);
	dcam_dphy_write(d, 0x54, 0x14);
	dcam_dphy_write(d, 0x64, 0x14);
	dcam_dphy_write(d, 0x74, 0x14);
	writel(0, d->csi + CSI_TST_CTRL1);
	dcam_rmw(d->csi + CSI_TST_CTRL0, 0, TST_CLK);
	udelay(1);
	writel(TST_EN, d->csi + CSI_TST_CTRL1);
	dcam_rmw(d->csi + CSI_TST_CTRL0, TST_CLK, 0);
	udelay(1);
	writel(0, d->csi + CSI_TST_CTRL1);

	writel(s->lanes - 1, d->csi + CSI_N_LANES);
	writel(1, d->csi + CSI_PHY_SHUTDOWNZ);
	udelay(1);
	writel(1, d->csi + CSI_DPHY_RSTZ);
	writel(0, d->csi + CSI_RESETN);
	udelay(1);
	writel(1, d->csi + CSI_RESETN);
	/* errors latch anyway; read them with debug output */
	writel(~0u, d->csi + CSI_MASK1);
	writel(~0u, d->csi + CSI_MASK2);
	usleep_range(1000, 2000);
	dev_dbg(d->dev, "CSI-2 %u lane(s) at %u Mbit/s, PHY_STATE %#x\n",
		s->lanes, mbps, readl(d->csi + CSI_PHY_STATE));
	return 0;
}

static void dcam_csi_stop(struct dcam *d, struct dcam_sensor *s)
{
	writel(0, d->csi + CSI_RESETN);
	writel(0, d->csi + CSI_DPHY_RSTZ);
	writel(0, d->csi + CSI_PHY_SHUTDOWNZ);
	if (s->port == DCAM_PORT_PHY_A)
		dcam_rmw(d->aon + AON_PWR_CTRL, 0, PWR_CSI0_PHY_PD);
}

/* --- DCAM ------------------------------------------------------------- */

static void dcam_set_addr(struct dcam *d, dma_addr_t addr)
{
	writel(addr, d->base + d->path->addr_y);
	writel(addr + d->fmt.width * d->fmt.height,
	       d->base + d->path->addr_uv);
}

static const struct dcam_path *dcam_pick_path(struct dcam *d)
{
	if (path_sel == 1 || path_sel == 2)
		return &dcam_paths[path_sel - 1];
	return &dcam_paths[d->fmt.width > DCAM_PATH1_MAX_WIDTH];
}

static dma_addr_t dcam_buf_addr(struct dcam *d, struct dcam_buf *buf)
{
	return buf ? vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0) :
		     d->scratch_dma;
}

/* under qlock: the next queued buffer or the scratch buffer */
static struct dcam_buf *dcam_take_buf(struct dcam *d)
{
	struct dcam_buf *buf;

	buf = list_first_entry_or_null(&d->queued, struct dcam_buf, list);
	if (buf)
		list_del(&buf->list);
	return buf;
}

/*
 * Scaler coefficients for YUV422 in, YUV420 out at 1:1, from the vendor's
 * Dcam_GenScaleCoeff() (drivers/media/sprd_dcam/common/gen_scale_coef.c).
 * They do not depend on the frame size at 1:1 (checked for 320x240 up to
 * 2048x1536); taps 4 luma, 4 chroma. Entries not listed are 0.
 */
#define DCAM_SC_TAP		4

static const u32 dcam_sc_coef_h[48] = {
	[1] = 0x00000200, 0x000000ff, 0x00000000, 0x00000400, 0x000041f7, 0x0003d2fc,
	0x0003fe03, 0x000009ff, 0x000093eb, 0x0003b6eb, 0x0003fc05, 0x00000ffe,
	0x000117da, 0x0003b4b8, 0x0003fc06, 0x00000ffe, 0x000147d8, 0x0003b0a3,
	0x0003fc07, 0x00000dfe, 0x000171da, 0x0003b48b, 0x0003fc07, 0x00000bfe,
	0x0001d9db, 0x0003d648, 0x0003fe04, 0x000007ff, 0x0001fbe9, 0x0003ee1f,
	0x00000002, 0x00000200, 0x000000ff, 0x00002bff, 0x0003f6f1, 0x00006bfd,
	0x0003eed7, 0x0000ddf8, 0x0003eca4, 0x000117f5, 0x0003ea8b, 0x000147f6,
	0x0003f06f, 0x0001aff7, 0x0003fa35, 0x0001e3fb, 0x0003fe15,
};

static const u32 dcam_sc_coef_v[132] = {
	[1] = 0x000000ff, 0x00000001, 0x00000000, 0xfffffffb, 0x000000f1, 0x00000015,
	0xffffffff, 0xfffffff7, 0x000000d7, 0x00000035, 0xfffffffd, 0xfffffff6,
	0x000000a4, 0x0000006e, 0xfffffff8, 0xfffffff5, 0x0000008b, 0x0000008b,
	0xfffffff5, 0xfffffff8, 0x0000006f, 0x000000a3, 0xfffffff6, 0xfffffffd,
	0x00000035, 0x000000d7, 0xfffffff7, 0xffffffff, 0x00000015, 0x000000f1,
	0xfffffffb,
};

static const u32 dcam_sc_coef_vc[132] = {
	[0] = 0x00000034, 0x00000098, 0x00000034,
	[16] = 0x00000015, 0x000000b0, 0x0000003a, 0x00000001,
	[32] = 0x00000011, 0x000000a3, 0x0000004a, 0x00000002,
	[48] = 0x0000000d, 0x0000008f, 0x0000005f, 0x00000005,
	[64] = 0x00000009, 0x00000077, 0x00000077, 0x00000009,
	[80] = 0x00000005, 0x0000005f, 0x0000008f, 0x0000000d,
	[96] = 0x00000002, 0x0000004b, 0x000000a2, 0x00000011,
	[112] = 0x00000001, 0x0000003a, 0x000000b0, 0x00000015,
	[128] = 0x000000cc, 0x00000000, 0x00000000, 0x00000034,
};

static void dcam_load_coef(struct dcam *d, u32 off, const u32 *coef,
			   unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		writel(coef[i], d->base + off + 4 * i);
}

static void dcam_hw_start(struct dcam *d)
{
	const struct dcam_path *p = d->path;
	void __iomem *regs = d->base + p->regs;
	u32 size = d->fmt.width | d->fmt.height << 16;
	u32 out = size;
	unsigned long flags;
	u32 v;

	readl_poll_timeout(d->base + AHBM_STS, v, !(v & AHBM_BUSY), 10, 10000);
	dcam_rmw(d->mmahb + MM_AHB_RST, 0, RST_DCAM | RST_CCIR);
	udelay(1);
	dcam_rmw(d->mmahb + MM_AHB_RST, RST_DCAM | RST_CCIR, 0);
	writel(INT_ALL, d->base + INT_CLR);

	/* the path's source is the capture block (0), not the ISP */
	dcam_rmw(d->base + DCAM_CFG, p->cfg_src, CFG_CSI2_IN);
	writel(CAP_MULTI_FRAME | FIELD_PREP(CAP_YUV_ORDER, 2),
	       d->base + CAP_MIPI_CTRL);
	writel(FIELD_PREP(CAP_PRE_SKIP, d->skip) | CAP_CLR_COUNT,
	       d->base + CAP_MIPI_FRM_CTRL);
	writel(0, d->base + CAP_MIPI_START);
	writel((d->fmt.width - 1) | (d->fmt.height - 1) << 16,
	       d->base + CAP_MIPI_END);
	writel(0, d->base + CAP_MIPI_IMG_DECI);

	/* trim = destination keeps the scale at 1:1 */
	if (p->id == 2 && path2_crop && path2_crop < d->fmt.height / 2)
		out = d->fmt.width | (d->fmt.height - (path2_crop & ~1)) << 16;
	writel(size, regs + PATH_SRC_SIZE);
	writel(0, regs + PATH_TRIM_START);
	writel(out, regs + PATH_TRIM_SIZE);
	writel(out, regs + PATH_DST_SIZE);
	if (d->fmt.pixelformat == V4L2_PIX_FMT_NV12) {
		/* the scaler at 1:1 does the 4:2:0 chroma decimation */
		dcam_load_coef(d, p->coef + SC_COEF_H, dcam_sc_coef_h,
			       ARRAY_SIZE(dcam_sc_coef_h));
		dcam_load_coef(d, p->coef + SC_COEF_V, dcam_sc_coef_v,
			       ARRAY_SIZE(dcam_sc_coef_v));
		dcam_load_coef(d, p->coef + SC_COEF_VC, dcam_sc_coef_vc,
			       ARRAY_SIZE(dcam_sc_coef_vc));
		writel(FIELD_PREP(P_OUT_FMT, 1) |
		       FIELD_PREP(P_UV_TAP, DCAM_SC_TAP) |
		       FIELD_PREP(P_Y_TAP, DCAM_SC_TAP), regs + PATH_CFG);
	} else {
		writel(P_SCALER_BYPASS, regs + PATH_CFG);
	}
	dcam_rmw(d->base + ENDIAN_SEL, 0, p->endian_little | ENDIAN_AXI);

	spin_lock_irqsave(&d->qlock, flags);
	d->cur = dcam_take_buf(d);
	d->next = NULL;
	d->next_valid = false;
	d->sequence = 0;
	d->errors = 0;
	dcam_set_addr(d, dcam_buf_addr(d, d->cur));
	spin_unlock_irqrestore(&d->qlock, flags);

	writel(p->int_done | p->int_sof | d->int_errors, d->base + INT_MASK);
	enable_irq(d->irq);
	dcam_rmw(d->base + AHBM_STS, 0, AHBM_WR_ARBIT);
	dcam_rmw(d->base + DCAM_CONTROL, 0, p->force_copy | p->coef_force);
	dcam_rmw(d->base + DCAM_CFG, 0, p->cfg_en);
	dcam_rmw(d->base + DCAM_CONTROL, 0, CTL_CAP_FORCE_COPY);
	dcam_rmw(d->base + DCAM_CONTROL, 0, CTL_CAP_EN);
}

static void dcam_hw_stop(struct dcam *d)
{
	const struct dcam_path *p = d->path;
	u32 v;

	writel(0, d->base + INT_MASK);
	dcam_rmw(d->base + DCAM_CONTROL, CTL_CAP_EN, 0);
	dcam_rmw(d->base + AHBM_STS, 0, p->ahbm_stop);
	udelay(10);
	dcam_rmw(d->base + DCAM_CFG, p->cfg_en, 0);
	dcam_rmw(d->base + DCAM_CONTROL, 0, p->force_copy);
	readl_poll_timeout(d->base + AHBM_STS, v, !(v & p->ahbm_busy),
			   10, 10000);
	dcam_rmw(d->mmahb + MM_AHB_RST, 0, p->rst);
	udelay(1);
	dcam_rmw(d->mmahb + MM_AHB_RST, p->rst, 0);
	writel(INT_ALL, d->base + INT_CLR);
	disable_irq(d->irq);
}

static void dcam_buf_complete(struct dcam *d, struct dcam_buf *buf,
			      enum vb2_buffer_state state)
{
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = d->sequence;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&buf->vb.vb2_buf, state);
}

static irqreturn_t dcam_irq(int irq, void *data)
{
	struct dcam *d = data;
	const struct dcam_path *p = d->path;
	u32 sts = readl(d->base + INT_STS);

	if (!sts)
		return IRQ_NONE;
	writel(sts, d->base + INT_CLR);

	spin_lock(&d->qlock);
	if (sts & d->int_errors) {
		if (!d->errors++)
			dev_warn(d->dev, "capture error %#x, CSI ERR1 %#x\n",
				 sts, readl(d->csi + CSI_ERR1));
	}
	/* the frame in cur is complete */
	if (sts & p->int_done) {
		if (d->cur)
			dcam_buf_complete(d, d->cur, sts & d->int_errors ?
					  VB2_BUF_STATE_ERROR :
					  VB2_BUF_STATE_DONE);
		d->cur = NULL;
		d->sequence++;
	}
	/* a new frame starts: it uses what latched; queue the one after */
	if (sts & p->int_sof) {
		if (d->next_valid) {
			/* no DONE for the previous frame: dropped, reuse it */
			if (d->cur)
				list_add(&d->cur->list, &d->queued);
			d->cur = d->next;
			d->next_valid = false;
		}
		d->next = dcam_take_buf(d);
		d->next_valid = true;
		dcam_set_addr(d, dcam_buf_addr(d, d->next));
		dcam_rmw(d->base + DCAM_CONTROL, 0, p->auto_copy);
	}
	spin_unlock(&d->qlock);
	return IRQ_HANDLED;
}

/* --- vb2 ------------------------------------------------------------- */

static int dcam_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct dcam *d = vb2_get_drv_priv(q);

	if (*nplanes)
		return sizes[0] < d->fmt.sizeimage ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = d->fmt.sizeimage;
	return 0;
}

static int dcam_buf_prepare(struct vb2_buffer *vb)
{
	struct dcam *d = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < d->fmt.sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, d->fmt.sizeimage);
	return 0;
}

static void dcam_buf_queue(struct vb2_buffer *vb)
{
	struct dcam *d = vb2_get_drv_priv(vb->vb2_queue);
	struct dcam_buf *buf = container_of(to_vb2_v4l2_buffer(vb),
					    struct dcam_buf, vb);
	unsigned long flags;

	spin_lock_irqsave(&d->qlock, flags);
	list_add_tail(&buf->list, &d->queued);
	spin_unlock_irqrestore(&d->qlock, flags);
}

static void dcam_return_bufs(struct dcam *d, enum vb2_buffer_state state)
{
	struct dcam_buf *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&d->qlock, flags);
	if (d->cur)
		vb2_buffer_done(&d->cur->vb.vb2_buf, state);
	if (d->next_valid && d->next)
		vb2_buffer_done(&d->next->vb.vb2_buf, state);
	d->cur = d->next = NULL;
	d->next_valid = false;
	list_for_each_entry_safe(buf, tmp, &d->queued, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&d->qlock, flags);
}

/* the sensor format must match the video node */
static int dcam_check_format(struct dcam *d, struct dcam_sensor *s)
{
	struct v4l2_subdev_format fmt = { .which = V4L2_SUBDEV_FORMAT_ACTIVE };
	struct media_pad *pad;
	int ret;

	pad = media_pad_remote_pad_first(&d->pad);
	fmt.pad = pad->index;
	ret = v4l2_subdev_call_state_active(s->sd, pad, get_fmt, &fmt);
	if (ret)
		return ret;
	if (fmt.format.code != MEDIA_BUS_FMT_UYVY8_1X16 ||
	    fmt.format.width != d->fmt.width ||
	    fmt.format.height != d->fmt.height) {
		dev_dbg(d->dev, "sensor %ux%u %#x, video node %ux%u\n",
			fmt.format.width, fmt.format.height, fmt.format.code,
			d->fmt.width, d->fmt.height);
		return -EPIPE;
	}
	return 0;
}

static int dcam_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct dcam *d = vb2_get_drv_priv(q);
	struct dcam_sensor *s = NULL;
	struct media_pad *remote;
	unsigned int i;
	s64 link_freq;
	u32 skip = 2;
	int ret;

	remote = media_pad_remote_pad_first(&d->pad);
	for (i = 0; remote && i < DCAM_NUM_PORTS; i++)
		if (d->sensors[i].sd &&
		    &d->sensors[i].sd->entity == remote->entity)
			s = &d->sensors[i];
	if (!s) {
		ret = -ENOLINK;
		goto err_bufs;
	}

	ret = video_device_pipeline_start(&d->vdev, &d->pipe);
	if (ret)
		goto err_bufs;
	ret = dcam_check_format(d, s);
	if (ret)
		goto err_pipe;

	d->scratch_size = d->fmt.sizeimage;
	d->scratch = dma_alloc_coherent(d->dev, d->scratch_size,
					&d->scratch_dma, GFP_KERNEL);
	if (!d->scratch) {
		ret = -ENOMEM;
		goto err_pipe;
	}

	ret = pm_runtime_resume_and_get(d->dev);
	if (ret)
		goto err_scratch;

	/*
	 * Sensor configured and asleep: lanes in LP-11 for the D-PHY. A
	 * sensor that must switch modes while streaming (S5K4ECGX stills)
	 * streams already; the vendor always started the PHY like that.
	 */
	ret = v4l2_subdev_call(s->sd, video, pre_streamon,
			       V4L2_SUBDEV_PRE_STREAMON_FL_MANUAL_LP);
	if (ret && ret != -ENOIOCTLCMD)
		goto err_pm;

	link_freq = v4l2_get_link_freq(remote, 16, 2 * s->lanes);
	if (link_freq < 0) {
		ret = link_freq;
		goto err_post;
	}
	ret = dcam_csi_start(d, s, link_freq);
	if (ret)
		goto err_post;

	v4l2_subdev_call(s->sd, sensor, g_skip_frames, &skip);
	d->skip = min(skip, 15U);
	d->path = dcam_pick_path(d);
	d->int_errors = d->path->int_ov | INT_SN_LINE_ERR | INT_SN_FRAME_ERR |
			INT_MIPI_OV;
	/* test build: anything but the proven preview setup */
	if (d->path->id != 1 || d->fmt.width > DCAM_PATH1_MAX_WIDTH ||
	    d->skip != 2)
		dev_info(d->dev, "%ux%u through path %u, skip %u, PHY_STATE %#x\n",
			 d->fmt.width, d->fmt.height, d->path->id, d->skip,
			 readl(d->csi + CSI_PHY_STATE));

	d->active = s;
	dcam_hw_start(d);
	ret = v4l2_subdev_enable_streams(s->sd, remote->index, BIT(0));
	if (ret)
		goto err_hw;
	return 0;

err_hw:
	dcam_hw_stop(d);
	dcam_csi_stop(d, s);
	d->active = NULL;
err_post:
	v4l2_subdev_call(s->sd, video, post_streamoff);
err_pm:
	pm_runtime_put(d->dev);
err_scratch:
	dma_free_coherent(d->dev, d->scratch_size, d->scratch, d->scratch_dma);
	d->scratch = NULL;
err_pipe:
	video_device_pipeline_stop(&d->vdev);
err_bufs:
	dcam_return_bufs(d, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void dcam_stop_streaming(struct vb2_queue *q)
{
	struct dcam *d = vb2_get_drv_priv(q);
	struct dcam_sensor *s = d->active;
	struct media_pad *remote = media_pad_remote_pad_first(&d->pad);

	if (s) {
		v4l2_subdev_disable_streams(s->sd, remote->index, BIT(0));
		dcam_hw_stop(d);
		dcam_csi_stop(d, s);
		v4l2_subdev_call(s->sd, video, post_streamoff);
		d->active = NULL;
		pm_runtime_put(d->dev);
	}
	if (d->errors)
		dev_info(d->dev, "%u capture errors\n", d->errors);
	dcam_return_bufs(d, VB2_BUF_STATE_ERROR);
	dma_free_coherent(d->dev, d->scratch_size, d->scratch, d->scratch_dma);
	d->scratch = NULL;
	video_device_pipeline_stop(&d->vdev);
}

static const struct vb2_ops dcam_vb2_ops = {
	.queue_setup = dcam_queue_setup,
	.buf_prepare = dcam_buf_prepare,
	.buf_queue = dcam_buf_queue,
	.start_streaming = dcam_start_streaming,
	.stop_streaming = dcam_stop_streaming,
};

/* --- video node -------------------------------------------------------- */

/*
 * path1 output: Y plane plus CbCr plane, 4:2:0 or 4:2:2. With the scaler
 * bypassed the vendor only uses 4:2:2; 4:2:0 works as well (the DMA
 * writes exactly w * h * 3 / 2 bytes) and is what most user space
 * handles (GStreamer's videocrop in camerabin knows no NV16), so it
 * comes first.
 */
static const u32 dcam_formats[] = { V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV16 };

static bool dcam_format_valid(u32 pixelformat)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dcam_formats); i++)
		if (dcam_formats[i] == pixelformat)
			return true;
	return false;
}

static void dcam_fill_fmt(struct v4l2_pix_format *f)
{
	if (!dcam_format_valid(f->pixelformat))
		f->pixelformat = dcam_formats[0];
	f->width = clamp(ALIGN(f->width, 4), 16U, (u32)DCAM_MAX_WIDTH);
	f->height = clamp(ALIGN(f->height, 2), 16U, (u32)DCAM_MAX_HEIGHT);
	f->field = V4L2_FIELD_NONE;
	f->bytesperline = f->width;
	f->sizeimage = f->width * f->height;
	f->sizeimage += f->pixelformat == V4L2_PIX_FMT_NV12 ? f->sizeimage / 2 :
							       f->sizeimage;
	f->colorspace = V4L2_COLORSPACE_SRGB;
	f->ycbcr_enc = V4L2_YCBCR_ENC_601;
	f->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	f->xfer_func = V4L2_XFER_FUNC_SRGB;
}

static int dcam_querycap(struct file *file, void *priv,
			 struct v4l2_capability *cap)
{
	strscpy(cap->driver, "sc7730-dcam", sizeof(cap->driver));
	strscpy(cap->card, "SC7730 DCAM", sizeof(cap->card));
	return 0;
}

static int dcam_enum_fmt(struct file *file, void *priv,
			 struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(dcam_formats) ||
	    (f->mbus_code && f->mbus_code != MEDIA_BUS_FMT_UYVY8_1X16))
		return -EINVAL;
	f->pixelformat = dcam_formats[f->index];
	return 0;
}

static int dcam_enum_framesizes(struct file *file, void *priv,
				struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || !dcam_format_valid(fsize->pixel_format))
		return -EINVAL;
	/* as dcam_fill_fmt() aligns them */
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = 16;
	fsize->stepwise.max_width = DCAM_MAX_WIDTH;
	fsize->stepwise.step_width = 4;
	fsize->stepwise.min_height = 16;
	fsize->stepwise.max_height = DCAM_MAX_HEIGHT;
	fsize->stepwise.step_height = 2;
	return 0;
}

static int dcam_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct dcam *d = video_drvdata(file);

	f->fmt.pix = d->fmt;
	return 0;
}

static int dcam_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	dcam_fill_fmt(&f->fmt.pix);
	return 0;
}

static int dcam_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct dcam *d = video_drvdata(file);

	if (vb2_is_busy(&d->queue))
		return -EBUSY;
	dcam_fill_fmt(&f->fmt.pix);
	d->fmt = f->fmt.pix;
	return 0;
}

static int dcam_enum_input(struct file *file, void *priv,
			   struct v4l2_input *inp)
{
	if (inp->index)
		return -EINVAL;
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(inp->name, "Camera", sizeof(inp->name));
	return 0;
}

static int dcam_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int dcam_s_input(struct file *file, void *priv, unsigned int i)
{
	return i ? -EINVAL : 0;
}

static const struct v4l2_ioctl_ops dcam_ioctl_ops = {
	.vidioc_querycap = dcam_querycap,
	.vidioc_enum_fmt_vid_cap = dcam_enum_fmt,
	.vidioc_enum_framesizes = dcam_enum_framesizes,
	.vidioc_g_fmt_vid_cap = dcam_g_fmt,
	.vidioc_try_fmt_vid_cap = dcam_try_fmt,
	.vidioc_s_fmt_vid_cap = dcam_s_fmt,
	.vidioc_enum_input = dcam_enum_input,
	.vidioc_g_input = dcam_g_input,
	.vidioc_s_input = dcam_s_input,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations dcam_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static int dcam_link_validate(struct media_link *link)
{
	/* checked in start_streaming, once the format is known */
	return 0;
}

/* one sensor at a time: the other link must be disabled first */
static int dcam_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	struct media_link *link;

	if (!(flags & MEDIA_LNK_FL_ENABLED))
		return 0;
	list_for_each_entry(link, &entity->links, list)
		if (link->sink == local && link->source != remote &&
		    (link->flags & MEDIA_LNK_FL_ENABLED))
			return -EBUSY;
	return 0;
}

static const struct media_entity_operations dcam_entity_ops = {
	.link_setup = dcam_link_setup,
	.link_validate = dcam_link_validate,
};

/* --- async sensors --------------------------------------------------- */

static int dcam_notify_bound(struct v4l2_async_notifier *notifier,
			     struct v4l2_subdev *sd,
			     struct v4l2_async_connection *asc)
{
	struct dcam *d = container_of(notifier, struct dcam, notifier);
	struct dcam_asc *dasc = container_of(asc, struct dcam_asc, asc);
	struct dcam_sensor *s = &d->sensors[dasc->port];
	bool first = true;
	unsigned int i;
	int pad;

	for (i = 0; i < DCAM_NUM_PORTS; i++)
		if (d->sensors[i].sd)
			first = false;

	pad = media_entity_get_fwnode_pad(&sd->entity, sd->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0)
		pad = 0;

	s->sd = sd;
	s->port = dasc->port;
	s->lanes = dasc->lanes;
	/* one sensor at a time: the first one bound streams by default */
	return media_create_pad_link(&sd->entity, pad, &d->vdev.entity, 0,
				     first ? MEDIA_LNK_FL_ENABLED : 0);
}

static int dcam_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct dcam *d = container_of(notifier, struct dcam, notifier);

	return v4l2_device_register_subdev_nodes(&d->v4l2);
}

static const struct v4l2_async_notifier_operations dcam_notify_ops = {
	.bound = dcam_notify_bound,
	.complete = dcam_notify_complete,
};

static int dcam_parse_dt(struct dcam *d)
{
	unsigned int port;
	int ret;

	for (port = 0; port < DCAM_NUM_PORTS; port++) {
		struct v4l2_fwnode_endpoint ep = {
			.bus_type = V4L2_MBUS_CSI2_DPHY,
		};
		struct fwnode_handle *fw;
		struct dcam_asc *dasc;

		fw = fwnode_graph_get_endpoint_by_id(dev_fwnode(d->dev), port, 0,
						     FWNODE_GRAPH_ENDPOINT_NEXT);
		if (!fw)
			continue;
		ret = v4l2_fwnode_endpoint_parse(fw, &ep);
		if (ret || !ep.bus.mipi_csi2.num_data_lanes ||
		    ep.bus.mipi_csi2.num_data_lanes > (port ? 1 : 2)) {
			fwnode_handle_put(fw);
			return dev_err_probe(d->dev, ret ?: -EINVAL,
					     "port %u: bad endpoint\n", port);
		}
		dasc = v4l2_async_nf_add_fwnode_remote(&d->notifier, fw,
						       struct dcam_asc);
		fwnode_handle_put(fw);
		if (IS_ERR(dasc))
			return PTR_ERR(dasc);
		dasc->port = port;
		dasc->lanes = ep.bus.mipi_csi2.num_data_lanes;
	}
	return 0;
}

/* --- probe -------------------------------------------------------------- */

static int dcam_register_mclk(struct dcam *d)
{
	struct clk_init_data init = {
		.name = "dcam-mclk",
		.ops = &dcam_mclk_ops,
	};
	int ret;

	d->mclk.init = &init;
	ret = devm_clk_hw_register(d->dev, &d->mclk);
	if (ret)
		return ret;
	return devm_of_clk_add_hw_provider(d->dev, of_clk_hw_simple_get,
					   &d->mclk);
}

static void __iomem *dcam_map(struct platform_device *pdev, const char *name)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return IOMEM_ERR_PTR(-EINVAL);
	/* MM AHB, MM clock and AON APB are shared with other blocks */
	return devm_ioremap(&pdev->dev, res->start, resource_size(res)) ?:
	       IOMEM_ERR_PTR(-ENOMEM);
}

static int dcam_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct video_device *vdev;
	struct vb2_queue *q;
	struct dcam *d;
	int ret;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->dev = dev;
	platform_set_drvdata(pdev, d);
	mutex_init(&d->lock);
	spin_lock_init(&d->qlock);
	INIT_LIST_HEAD(&d->queued);

	d->base = devm_platform_ioremap_resource_byname(pdev, "dcam");
	if (IS_ERR(d->base))
		return PTR_ERR(d->base);
	d->csi = devm_platform_ioremap_resource_byname(pdev, "csi");
	if (IS_ERR(d->csi))
		return PTR_ERR(d->csi);
	d->mmahb = dcam_map(pdev, "mm-ahb");
	d->mmclk = dcam_map(pdev, "mm-clk");
	d->aon = dcam_map(pdev, "aon-apb");
	if (IS_ERR(d->mmahb) || IS_ERR(d->mmclk) || IS_ERR(d->aon))
		return dev_err_probe(dev, -EINVAL, "missing register regions\n");

	d->irq = platform_get_irq(pdev, 0);
	if (d->irq < 0)
		return d->irq;
	/* only while streaming: the registers are gone with the MM domain */
	ret = devm_request_irq(dev, d->irq, dcam_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), d);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	pm_runtime_enable(dev);

	ret = dcam_register_mclk(d);
	if (ret)
		goto err_pm;

	d->fmt.width = 800;
	d->fmt.height = 600;
	dcam_fill_fmt(&d->fmt);

	d->mdev.dev = dev;
	strscpy(d->mdev.model, "SC7730 DCAM", sizeof(d->mdev.model));
	media_device_init(&d->mdev);
	d->v4l2.mdev = &d->mdev;
	ret = v4l2_device_register(dev, &d->v4l2);
	if (ret)
		goto err_mdev;

	q = &d->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->drv_priv = d;
	q->buf_struct_size = sizeof(struct dcam_buf);
	q->ops = &dcam_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->lock = &d->lock;
	q->dev = dev;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_v4l2;

	d->pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	vdev = &d->vdev;
	vdev->entity.ops = &dcam_entity_ops;
	ret = media_entity_pads_init(&vdev->entity, 1, &d->pad);
	if (ret)
		goto err_v4l2;
	strscpy(vdev->name, "sc7730-dcam", sizeof(vdev->name));
	vdev->fops = &dcam_fops;
	vdev->ioctl_ops = &dcam_ioctl_ops;
	vdev->v4l2_dev = &d->v4l2;
	vdev->queue = q;
	vdev->lock = &d->lock;
	vdev->release = video_device_release_empty;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE | V4L2_CAP_IO_MC;
	video_set_drvdata(vdev, d);
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_entity;

	v4l2_async_nf_init(&d->notifier, &d->v4l2);
	d->notifier.ops = &dcam_notify_ops;
	ret = dcam_parse_dt(d);
	if (ret)
		goto err_vdev;
	ret = v4l2_async_nf_register(&d->notifier);
	if (ret)
		goto err_nf;

	ret = media_device_register(&d->mdev);
	if (ret)
		goto err_nf_unreg;
	return 0;

err_nf_unreg:
	v4l2_async_nf_unregister(&d->notifier);
err_nf:
	v4l2_async_nf_cleanup(&d->notifier);
err_vdev:
	video_unregister_device(vdev);
err_entity:
	media_entity_cleanup(&vdev->entity);
err_v4l2:
	v4l2_device_unregister(&d->v4l2);
err_mdev:
	media_device_cleanup(&d->mdev);
err_pm:
	pm_runtime_disable(dev);
	return ret;
}

static void dcam_remove(struct platform_device *pdev)
{
	struct dcam *d = platform_get_drvdata(pdev);

	media_device_unregister(&d->mdev);
	v4l2_async_nf_unregister(&d->notifier);
	v4l2_async_nf_cleanup(&d->notifier);
	video_unregister_device(&d->vdev);
	media_entity_cleanup(&d->vdev.entity);
	v4l2_device_unregister(&d->v4l2);
	media_device_cleanup(&d->mdev);
	pm_runtime_disable(&pdev->dev);
}

static const struct dev_pm_ops dcam_pm_ops = {
	RUNTIME_PM_OPS(dcam_runtime_suspend, dcam_runtime_resume, NULL)
};

static const struct of_device_id dcam_of_match[] = {
	{ .compatible = "sprd,sc7730-dcam" },
	{ }
};
MODULE_DEVICE_TABLE(of, dcam_of_match);

static struct platform_driver dcam_driver = {
	.probe = dcam_probe,
	.remove = dcam_remove,
	.driver = {
		.name = "sc7730-dcam",
		.of_match_table = dcam_of_match,
		.pm = pm_ptr(&dcam_pm_ops),
	},
};
module_platform_driver(dcam_driver);

MODULE_DESCRIPTION("Spreadtrum SC7730 camera capture (DCAM + CSI-2)");
MODULE_LICENSE("GPL");
