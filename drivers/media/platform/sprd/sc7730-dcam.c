// SPDX-License-Identifier: GPL-2.0
/*
 * Camera capture of the Spreadtrum SC7730/SC8830 ("shark"): the Synopsys
 * DesignWare CSI-2 host (v1.03) with its two D-PHYs and the DCAM capture
 * engine, used here as a plain receiver for sensors with their own ISP.
 *
 * DCAM path 1 writes the captured YUV422 with the scaler bypassed as a Y
 * plane plus an interleaved CbCr plane (NV16); it has no line pitch, so
 * lines are packed and the CbCr plane starts at width * height. Frame
 * addresses are shadowed: written at the path's start of frame they latch
 * at the next one (auto copy), as in the vendor driver. A scratch buffer
 * takes frames for which user space has queued nothing.
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
#define  CFG_PATH1_EN		BIT(1)
#define  CFG_CSI2_IN		BIT(9)
#define  CFG_PATH1_SRC		GENMASK(12, 11)
#define DCAM_CONTROL		0x04
#define  CTL_CAP_FORCE_COPY	BIT(0)
#define  CTL_CAP_EN		BIT(2)
#define  CTL_PATH1_FORCE_COPY	BIT(10)
#define  CTL_PATH1_AUTO_COPY	BIT(11)
#define  CTL_PATH1_COEF_FORCE	BIT(14)
#define PATH1_CFG		0x10
#define  P1_SCALER_BYPASS	BIT(20)
#define PATH1_SRC_SIZE		0x14
#define PATH1_DST_SIZE		0x18
#define PATH1_TRIM_START	0x1c
#define PATH1_TRIM_SIZE		0x20
#define INT_STS			0x3c
#define INT_MASK		0x40
#define INT_CLR			0x44
#define  INT_PATH1_DONE		BIT(6)
#define  INT_PATH1_OV		BIT(7)
#define  INT_SN_LINE_ERR	BIT(10)
#define  INT_SN_FRAME_ERR	BIT(11)
#define  INT_MIPI_OV		BIT(14)
#define  INT_PATH1_SOF		BIT(19)
#define  INT_ALL		GENMASK(20, 0)
#define  INT_ERRORS		(INT_PATH1_OV | INT_SN_LINE_ERR | \
				 INT_SN_FRAME_ERR | INT_MIPI_OV)
#define FRM_ADDR1		0x50	/* path1 Y */
#define FRM_ADDR2		0x54	/* path1 CbCr */
#define ENDIAN_SEL		0x6c
#define  ENDIAN_P1_LITTLE	(BIT(6) | BIT(8))
#define  ENDIAN_AXI		(BIT(18) | BIT(19))
#define AHBM_STS		0x70
#define  AHBM_BUSY		BIT(0)
#define  AHBM_PATH1_STOP	BIT(4)
#define  AHBM_WR_ARBIT		BIT(8)
#define  AHBM_PATH1_BUSY	BIT(17)
#define CAP_MIPI_CTRL		0x128
#define  CAP_MULTI_FRAME	BIT(6)
#define  CAP_YUV_ORDER		GENMASK(8, 7)	/* 2 = UYVY */
#define CAP_MIPI_FRM_CTRL	0x12c
#define  CAP_PRE_SKIP		GENMASK(3, 0)
#define  CAP_CLR_COUNT		BIT(22)
#define CAP_MIPI_START		0x130
#define CAP_MIPI_END		0x134
#define CAP_MIPI_IMG_DECI	0x138

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
#define  RST_PATH1		BIT(8)
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
#define DCAM_MAX_WIDTH		2048	/* path1 line buffer */
#define DCAM_MAX_HEIGHT		4092

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
	writel(addr, d->base + FRM_ADDR1);
	writel(addr + d->fmt.width * d->fmt.height, d->base + FRM_ADDR2);
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

static void dcam_hw_start(struct dcam *d)
{
	u32 size = d->fmt.width | d->fmt.height << 16;
	unsigned long flags;
	u32 v;

	readl_poll_timeout(d->base + AHBM_STS, v, !(v & AHBM_BUSY), 10, 10000);
	dcam_rmw(d->mmahb + MM_AHB_RST, 0, RST_DCAM | RST_CCIR);
	udelay(1);
	dcam_rmw(d->mmahb + MM_AHB_RST, RST_DCAM | RST_CCIR, 0);
	writel(INT_ALL, d->base + INT_CLR);

	dcam_rmw(d->base + DCAM_CFG, CFG_PATH1_SRC, CFG_CSI2_IN);
	writel(CAP_MULTI_FRAME | FIELD_PREP(CAP_YUV_ORDER, 2),
	       d->base + CAP_MIPI_CTRL);
	writel(FIELD_PREP(CAP_PRE_SKIP, 2) | CAP_CLR_COUNT,
	       d->base + CAP_MIPI_FRM_CTRL);
	writel(0, d->base + CAP_MIPI_START);
	writel((d->fmt.width - 1) | (d->fmt.height - 1) << 16,
	       d->base + CAP_MIPI_END);
	writel(0, d->base + CAP_MIPI_IMG_DECI);

	writel(size, d->base + PATH1_SRC_SIZE);
	writel(0, d->base + PATH1_TRIM_START);
	writel(size, d->base + PATH1_TRIM_SIZE);
	writel(size, d->base + PATH1_DST_SIZE);
	writel(P1_SCALER_BYPASS, d->base + PATH1_CFG);	/* YUV422 2-plane */
	dcam_rmw(d->base + ENDIAN_SEL, 0, ENDIAN_P1_LITTLE | ENDIAN_AXI);

	spin_lock_irqsave(&d->qlock, flags);
	d->cur = dcam_take_buf(d);
	d->next = NULL;
	d->next_valid = false;
	d->sequence = 0;
	d->errors = 0;
	dcam_set_addr(d, dcam_buf_addr(d, d->cur));
	spin_unlock_irqrestore(&d->qlock, flags);

	writel(INT_PATH1_DONE | INT_PATH1_SOF | INT_ERRORS, d->base + INT_MASK);
	enable_irq(d->irq);
	dcam_rmw(d->base + AHBM_STS, 0, AHBM_WR_ARBIT);
	dcam_rmw(d->base + DCAM_CONTROL, 0,
		 CTL_PATH1_FORCE_COPY | CTL_PATH1_COEF_FORCE);
	dcam_rmw(d->base + DCAM_CFG, 0, CFG_PATH1_EN);
	dcam_rmw(d->base + DCAM_CONTROL, 0, CTL_CAP_FORCE_COPY);
	dcam_rmw(d->base + DCAM_CONTROL, 0, CTL_CAP_EN);
}

static void dcam_hw_stop(struct dcam *d)
{
	u32 v;

	writel(0, d->base + INT_MASK);
	dcam_rmw(d->base + DCAM_CONTROL, CTL_CAP_EN, 0);
	dcam_rmw(d->base + AHBM_STS, 0, AHBM_PATH1_STOP);
	udelay(10);
	dcam_rmw(d->base + DCAM_CFG, CFG_PATH1_EN, 0);
	dcam_rmw(d->base + DCAM_CONTROL, 0, CTL_PATH1_FORCE_COPY);
	readl_poll_timeout(d->base + AHBM_STS, v, !(v & AHBM_PATH1_BUSY),
			   10, 10000);
	dcam_rmw(d->mmahb + MM_AHB_RST, 0, RST_PATH1);
	udelay(1);
	dcam_rmw(d->mmahb + MM_AHB_RST, RST_PATH1, 0);
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
	u32 sts = readl(d->base + INT_STS);

	if (!sts)
		return IRQ_NONE;
	writel(sts, d->base + INT_CLR);

	spin_lock(&d->qlock);
	if (sts & INT_ERRORS) {
		if (!d->errors++)
			dev_warn(d->dev, "capture error %#x, CSI ERR1 %#x\n",
				 sts, readl(d->csi + CSI_ERR1));
	}
	/* the frame in cur is complete */
	if (sts & INT_PATH1_DONE) {
		if (d->cur)
			dcam_buf_complete(d, d->cur, sts & INT_ERRORS ?
					  VB2_BUF_STATE_ERROR :
					  VB2_BUF_STATE_DONE);
		d->cur = NULL;
		d->sequence++;
	}
	/* a new frame starts: it uses what latched; queue the one after */
	if (sts & INT_PATH1_SOF) {
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
		dcam_rmw(d->base + DCAM_CONTROL, 0, CTL_PATH1_AUTO_COPY);
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

	/* sensor configured and asleep: lanes in LP-11 for the D-PHY */
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

static void dcam_fill_fmt(struct v4l2_pix_format *f)
{
	f->pixelformat = V4L2_PIX_FMT_NV16;
	f->width = clamp(ALIGN(f->width, 4), 16U, (u32)DCAM_MAX_WIDTH);
	f->height = clamp(ALIGN(f->height, 2), 16U, (u32)DCAM_MAX_HEIGHT);
	f->field = V4L2_FIELD_NONE;
	f->bytesperline = f->width;
	f->sizeimage = f->width * f->height * 2;
	f->colorspace = V4L2_COLORSPACE_SRGB;
	f->ycbcr_enc = V4L2_YCBCR_ENC_601;
	f->quantization = V4L2_QUANTIZATION_LIM_RANGE;
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
	if (f->index || (f->mbus_code && f->mbus_code != MEDIA_BUS_FMT_UYVY8_1X16))
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_NV16;
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

static const struct media_entity_operations dcam_entity_ops = {
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
