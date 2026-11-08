// SPDX-License-Identifier: GPL-2.0
/*
 * Display controller (DISPC) of the Spreadtrum SC7730/SC8830 ("shark")
 * SoCs, taking over the pipeline that the bootloader leaves running.
 *
 * The bootloader powers and clocks DISPC, the DesignWare MIPI DSI host and
 * D-PHY, initialises the panel and scans out its splash buffer through the
 * OSD layer in DPI (video) mode. This driver keeps all of that as it is and
 * only moves the OSD base address to its own buffers, as the vendor kernel
 * does when it takes over the splash screen. Apart from the panel power-off
 * below it never stops DISPC, and it never touches clocks, resets or the
 * DSI configuration. Disabling the CRTC switches the backlight off and
 * points the layer at a black buffer, so freed memory is never scanned out.
 *
 * With panel_power_off set (the default) it also stops DISPC (without a
 * reset, as the vendor's ESD path does), sends the NT51017 its sleep-in
 * packet over the DSI host in LP command mode and cuts the panel supplies.
 * panel_power_off=0 leaves DISPC and the panel running. Enabling powers
 * the panel up the way the bootloader does, which needs no init commands,
 * and restarts video mode and DISPC. The DSI host and D-PHY keep their
 * configuration throughout. The panel does not come back from sleep-in
 * with a sleep-out (0x11) packet alone, so the supplies are part of it.
 *
 * Layer registers are double-buffered: setting DPI_CTRL bit 5 requests the
 * transfer to the active copies at the next VSYNC, which raises
 * UPDATE_DONE, the page-flip completion. VSYNC itself drives vblank.
 * Register layout from the vendor 3.10 sprdfb driver and the bootloader,
 * see vendor-ref/research/dispc-report.md in the bring-up repository. Unlike
 * the mainline SharkL3 DPU (sprd_dpu.c) every offset differs.
 *
 * There is no IOMMU in front of DISPC, so buffers come from CMA. They are
 * mapped cached and flushed before each flip, as for software rendering
 * with uncached memory every blend would read across the bus.
 */

#include <linux/aperture.h>
#include <linux/backlight.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#define DISPC_CTRL		0x00
#define  CTRL_EN		BIT(0)
#define  CTRL_IF		GENMASK(2, 1)	/* 0 = DPI */
#define  CTRL_RUN		BIT(4)
#define DISPC_SIZE_XY		0x04
#define DISPC_OSD_CTRL		0x40
#define  OSD_CTRL_RGB8888	0x35		/* enable, block alpha, format 3 */
#define  OSD_CTRL_RB_SWITCH	BIT(15)
#define DISPC_OSD_BASE_ADDR	0x44
#define DISPC_OSD_SIZE_XY	0x48
#define DISPC_OSD_PITCH		0x4c		/* in pixels */
#define DISPC_OSD_DISP_XY	0x50
#define DISPC_INT_EN		0x70
#define DISPC_INT_CLR		0x74
#define DISPC_INT_STATUS	0x78
#define DISPC_INT_RAW		0x7c
#define  INT_ERR		BIT(2)		/* underflow */
#define  INT_UPDATE_DONE	BIT(4)
#define  INT_VSYNC		BIT(5)
#define  INT_ALL		GENMASK(5, 0)
#define DISPC_DPI_CTRL		0x80
#define  DPI_SW_UPDATE_ONLY	BIT(4)
#define  DPI_REG_UPDATE		BIT(5)
#define DISPC_DPI_H_TIMING	0x84
#define DISPC_DPI_V_TIMING	0x88
#define DISPC_DPI_STS1		0x90
#define  DPI_STS1_FRAME		GENMASK(15, 0)	/* +1 per frame */
#define  DPI_STS1_BUSY		BIT(16)
#define  DPI_SYNC		GENMASK(7, 0)
#define  DPI_BP			GENMASK(19, 8)
#define  DPI_FP			GENMASK(31, 20)

#define SIZE_W			GENMASK(11, 0)
#define SIZE_H			GENMASK(27, 16)

/* bootloader-owned gates and clocks, only read */
#define AP_AHB_EB		0x00
#define  AP_AHB_DISPC_EB	BIT(1)
#define AON_APB_EB1		0x04
#define  AON_DISP_EMC_EB	BIT(11)
#define AP_APB_EB		0x00
#define  AP_APB_CKG_EB		BIT(18)
#define AP_CLK_DISPC_DPI_CFG	0x34
#define  DPI_CLK_SEL		GENMASK(1, 0)
#define  DPI_CLK_DIV		GENMASK(15, 8)

/* DesignWare MIPI DSI host 1.21a, as the bootloader leaves it in video mode */
#define DSI_PCKHDL_CFG		0x2c
#define  PCKHDL_VIDEO		0x0f		/* EoTp tx/rx, BTA, ECC rx */
#define  PCKHDL_READ		0x0c		/* BTA, ECC rx: as sboot's ID read */
#define DSI_MODE_CFG		0x34
#define  MODE_CMD		BIT(0)
#define DSI_CMD_MODE_CFG	0x68
#define  CMD_MODE_ALL_LP	0x010f7f00
#define DSI_GEN_HDR		0x6c
#define  GEN_DCS_SHORT_WRITE_1	0x15		/* vendor type for its lists */
#define  GEN_DCS_READ		0x06
#define DSI_GEN_PLD_DATA	0x70
#define DSI_CMD_PKT_STATUS	0x74
#define  GEN_CMD_EMPTY		BIT(0)
#define  GEN_CMD_FULL		BIT(1)
#define  GEN_PLD_W_EMPTY	BIT(2)
#define  GEN_PLD_R_EMPTY	BIT(4)
#define  GEN_RD_CMD_BUSY	BIT(6)
#define DSI_LPCLK_CTRL		0x94
#define  LPCLK_HS		BIT(0)
#define DSI_INT_ST0		0xbc
#define DSI_INT_ST1		0xc0

#define NT51017_ID_REG		0x80
#define  NT51017_ID		0x03
#define NT51017_STATUS_REG	0x0a		/* 0x11 after power-up */
#define NT51017_SLEEP_IN	0x10

static bool panel_power_off = true;
module_param(panel_power_off, bool, 0644);
MODULE_PARM_DESC(panel_power_off,
		 "Stop DISPC and power the panel off while the CRTC is off (default: on)");

#define FLIP_TIMEOUT_MS		100

static const unsigned long sprd_dispc_dpi_parents[] = {
	128000000, 153600000, 192000000, 384000000
};

/* the only layer is opaque, block alpha 0xff as the bootloader set it */
static const u32 sprd_dispc_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
};

struct sprd_dispc {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct drm_display_mode mode;

	void __iomem *base;
	void __iomem *dsi;
	struct gpio_descs *panel_power;
	bool panel_off;
	struct backlight_device *backlight;

	/* never-freed black buffer to scan out while the CRTC is off */
	void *black;
	dma_addr_t black_dma;
	size_t fb_size;

	spinlock_t int_lock;		/* INT_EN */
	u32 int_en;
	dma_addr_t pending_addr;
	u32 pending_pitch;		/* in pixels, as OSD_PITCH */
	bool rb_switch;
	struct drm_pending_vblank_event *event;	/* under drm.event_lock */
	struct completion update_done;
	struct delayed_work flip_timeout;
};

static struct sprd_dispc *to_sprd_dispc(struct drm_device *drm)
{
	return container_of(drm, struct sprd_dispc, drm);
}

static void sprd_dispc_int_update(struct sprd_dispc *d, u32 clr, u32 set)
{
	unsigned long flags;

	spin_lock_irqsave(&d->int_lock, flags);
	d->int_en = (d->int_en & ~clr) | set;
	writel(d->int_en, d->base + DISPC_INT_EN);
	spin_unlock_irqrestore(&d->int_lock, flags);
}

/*
 * Request the latch of OSD_BASE_ADDR, OSD_PITCH (and OSD_CTRL) at the next
 * VSYNC. UPDATE_DONE is cleared first: clearing it after the request could
 * eat a latch that already happened.
 */
static void sprd_dispc_request_update(struct sprd_dispc *d, dma_addr_t addr,
				      u32 pitch)
{
	u32 ctrl = readl(d->base + DISPC_OSD_CTRL);

	if (!!(ctrl & OSD_CTRL_RB_SWITCH) != d->rb_switch)
		writel(ctrl ^ OSD_CTRL_RB_SWITCH, d->base + DISPC_OSD_CTRL);
	reinit_completion(&d->update_done);
	writel(INT_UPDATE_DONE, d->base + DISPC_INT_CLR);
	writel(addr, d->base + DISPC_OSD_BASE_ADDR);
	writel(pitch, d->base + DISPC_OSD_PITCH);
	writel(readl(d->base + DISPC_DPI_CTRL) | DPI_REG_UPDATE,
	       d->base + DISPC_DPI_CTRL);
	/* the vendor re-arms underflow reporting on every update */
	sprd_dispc_int_update(d, 0, INT_ERR);
}

/* flip and wait, for switching away from a buffer that is about to go */
static void sprd_dispc_flip_sync(struct sprd_dispc *d, dma_addr_t addr)
{
	sprd_dispc_request_update(d, addr, d->mode.hdisplay);
	if (!wait_for_completion_timeout(&d->update_done,
					 msecs_to_jiffies(FLIP_TIMEOUT_MS)))
		drm_warn(&d->drm, "no UPDATE_DONE, INT_RAW %08x DPI_CTRL %08x\n",
			 readl(d->base + DISPC_INT_RAW),
			 readl(d->base + DISPC_DPI_CTRL));
}

static void sprd_dispc_send_event(struct sprd_dispc *d)
{
	unsigned long flags;

	spin_lock_irqsave(&d->drm.event_lock, flags);
	if (d->event) {
		drm_crtc_send_vblank_event(&d->crtc, d->event);
		drm_crtc_vblank_put(&d->crtc);
		d->event = NULL;
	}
	spin_unlock_irqrestore(&d->drm.event_lock, flags);
}

static irqreturn_t sprd_dispc_irq(int irq, void *data)
{
	struct sprd_dispc *d = data;
	u32 sts = readl(d->base + DISPC_INT_STATUS);

	if (!sts)
		return IRQ_NONE;
	writel(sts, d->base + DISPC_INT_CLR);

	if (sts & INT_VSYNC)
		drm_crtc_handle_vblank(&d->crtc);
	if (sts & INT_UPDATE_DONE) {
		complete_all(&d->update_done);
		sprd_dispc_send_event(d);
	}
	if (sts & INT_ERR) {
		drm_warn_once(&d->drm, "underflow\n");
		sprd_dispc_int_update(d, INT_ERR, 0);
	}
	return IRQ_HANDLED;
}

/* do not let a compositor hang on a latch that never comes */
static void sprd_dispc_flip_timeout(struct work_struct *work)
{
	struct sprd_dispc *d = container_of(work, struct sprd_dispc,
					    flip_timeout.work);

	if (completion_done(&d->update_done))
		return;
	dev_warn_ratelimited(d->drm.dev,
			     "flip timed out, INT_RAW %08x DPI_CTRL %08x\n",
			     readl(d->base + DISPC_INT_RAW),
			     readl(d->base + DISPC_DPI_CTRL));
	sprd_dispc_send_event(d);
}

static int sprd_dispc_plane_atomic_check(struct drm_plane *plane,
					 struct drm_atomic_commit *state)
{
	struct drm_plane_state *new = drm_atomic_get_new_plane_state(state, plane);
	struct sprd_dispc *d = to_sprd_dispc(plane->dev);
	struct drm_crtc_state *crtc_state;
	unsigned int pitch;
	int ret;

	if (!new->crtc)
		return 0;
	crtc_state = drm_atomic_get_new_crtc_state(state, new->crtc);
	ret = drm_atomic_helper_check_plane_state(new, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, true);
	if (ret || !new->visible)
		return ret;

	/*
	 * One full-screen layer, the layer position and size stay as the
	 * bootloader set them. The pitch is per buffer: OSD_PITCH counts
	 * pixels (gralloc buffers need not be packed like dumb buffers).
	 */
	pitch = new->fb->pitches[0];
	if (new->crtc_x || new->crtc_y ||
	    new->crtc_w != d->mode.hdisplay || new->crtc_h != d->mode.vdisplay ||
	    pitch % 4 || pitch / 4 < d->mode.hdisplay ||
	    pitch / 4 > FIELD_MAX(SIZE_W)) {
		drm_dbg_atomic(plane->dev,
			       "[PLANE:%d] unsupported layer %ux%u+%d+%d, pitch %u\n",
			       plane->base.id, new->crtc_w, new->crtc_h,
			       new->crtc_x, new->crtc_y, pitch);
		return -EINVAL;
	}
	return 0;
}

static void sprd_dispc_plane_atomic_update(struct drm_plane *plane,
					   struct drm_atomic_commit *state)
{
	struct drm_plane_state *old = drm_atomic_get_old_plane_state(state, plane);
	struct drm_plane_state *new = drm_atomic_get_new_plane_state(state, plane);
	struct sprd_dispc *d = to_sprd_dispc(plane->dev);
	u32 format;

	if (!new->visible || !new->fb) {
		d->pending_addr = d->black_dma;
		d->pending_pitch = d->mode.hdisplay;
		return;
	}

	drm_fb_dma_sync_non_coherent(plane->dev, old, new);
	d->pending_addr = drm_fb_dma_get_gem_addr(new->fb, new, 0);
	d->pending_pitch = new->fb->pitches[0] / 4;
	format = new->fb->format->format;
	d->rb_switch = format == DRM_FORMAT_XBGR8888;
}

static void sprd_dispc_plane_atomic_disable(struct drm_plane *plane,
					    struct drm_atomic_commit *state)
{
	struct sprd_dispc *d = to_sprd_dispc(plane->dev);

	/* the old buffer may be freed after this returns */
	sprd_dispc_flip_sync(d, d->black_dma);
}

static const struct drm_plane_helper_funcs sprd_dispc_plane_helper_funcs = {
	.atomic_check = sprd_dispc_plane_atomic_check,
	.atomic_update = sprd_dispc_plane_atomic_update,
	.atomic_disable = sprd_dispc_plane_atomic_disable,
};

static const struct drm_plane_funcs sprd_dispc_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.atomic_create_state = drm_atomic_helper_plane_create_state,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static void sprd_dispc_crtc_atomic_flush(struct drm_crtc *crtc,
					 struct drm_atomic_commit *state)
{
	struct sprd_dispc *d = to_sprd_dispc(crtc->dev);
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (!crtc_state->active)
		return;

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc_state->event) {
		WARN_ON(drm_crtc_vblank_get(crtc));
		d->event = crtc_state->event;
		crtc_state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	sprd_dispc_request_update(d, d->pending_addr, d->pending_pitch);
	mod_delayed_work(system_dfl_wq, &d->flip_timeout,
			 msecs_to_jiffies(FLIP_TIMEOUT_MS));
}

/*
 * An enabled CRTC always needs a buffer to show (with panel_power_off
 * unset DISPC keeps scanning out); disabling all of it, as suspend and
 * shutdown do, needs none.
 */
static int sprd_dispc_crtc_atomic_check(struct drm_crtc *crtc,
					struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (!crtc_state->enable)
		return 0;
	return drm_atomic_helper_check_crtc_primary_plane(crtc_state);
}

static int sprd_dsi_wait(struct sprd_dispc *d, u32 mask, u32 want)
{
	u32 sts;
	int ret;

	ret = readl_poll_timeout(d->dsi + DSI_CMD_PKT_STATUS, sts,
				 (sts & mask) == want, 100, 20000);
	if (ret)
		drm_warn(&d->drm, "DSI command timeout, status %08x INT_ST1 %08x\n",
			 sts, readl(d->dsi + DSI_INT_ST1));
	return ret;
}

/* LP command mode with the HS clock off, vendor set_lp_mode */
static void sprd_dsi_cmd_mode(struct sprd_dispc *d)
{
	writel(MODE_CMD, d->dsi + DSI_MODE_CFG);
	writel(CMD_MODE_ALL_LP, d->dsi + DSI_CMD_MODE_CFG);
	writel(0, d->dsi + DSI_LPCLK_CTRL);
}

/* back to video, vendor order (no PWR_UP toggle on 1.21a) */
static void sprd_dsi_video_mode(struct sprd_dispc *d)
{
	writel(PCKHDL_VIDEO, d->dsi + DSI_PCKHDL_CFG);
	writel(LPCLK_HS, d->dsi + DSI_LPCLK_CTRL);
	writel(0, d->dsi + DSI_MODE_CFG);
	readl(d->dsi + DSI_INT_ST0);
	readl(d->dsi + DSI_INT_ST1);
}

static int sprd_dsi_write(struct sprd_dispc *d, u8 reg, u8 val)
{
	int ret = sprd_dsi_wait(d, GEN_CMD_FULL, 0);

	if (ret)
		return ret;
	readl(d->dsi + DSI_INT_ST0);
	readl(d->dsi + DSI_INT_ST1);
	writel(val << 16 | reg << 8 | GEN_DCS_SHORT_WRITE_1,
	       d->dsi + DSI_GEN_HDR);
	return sprd_dsi_wait(d, GEN_CMD_EMPTY | GEN_PLD_W_EMPTY,
			     GEN_CMD_EMPTY | GEN_PLD_W_EMPTY);
}

static int sprd_dsi_read(struct sprd_dispc *d, u8 reg)
{
	int ret;

	writel(PCKHDL_READ, d->dsi + DSI_PCKHDL_CFG);
	ret = sprd_dsi_wait(d, GEN_CMD_FULL, 0);
	if (ret)
		return ret;
	writel(reg << 8 | GEN_DCS_READ, d->dsi + DSI_GEN_HDR);
	ret = sprd_dsi_wait(d, GEN_RD_CMD_BUSY | GEN_PLD_R_EMPTY, 0);
	if (ret)
		return ret;
	return readl(d->dsi + DSI_GEN_PLD_DATA) & 0xff;
}

/* without a reset, as the vendor's ESD path (dispc_stop_for_feature) */
static void sprd_dispc_stop(struct sprd_dispc *d)
{
	u32 sts;

	writel(readl(d->base + DISPC_DPI_CTRL) | DPI_SW_UPDATE_ONLY,
	       d->base + DISPC_DPI_CTRL);
	writel(readl(d->base + DISPC_CTRL) & ~CTRL_RUN, d->base + DISPC_CTRL);
	if (readl_poll_timeout(d->base + DISPC_DPI_STS1, sts,
			       !(sts & DPI_STS1_BUSY), 100, 50000))
		drm_warn(&d->drm, "DISPC still busy, DPI_STS1 %08x\n", sts);
	udelay(25);
}

/* first-frame path of the vendor dispc_run; no IMG trick needed without a reset */
static void sprd_dispc_run(struct sprd_dispc *d)
{
	writel(readl(d->base + DISPC_DPI_CTRL) | DPI_REG_UPDATE,
	       d->base + DISPC_DPI_CTRL);
	udelay(30);
	writel(readl(d->base + DISPC_DPI_CTRL) & ~DPI_SW_UPDATE_ONLY,
	       d->base + DISPC_DPI_CTRL);
	writel(readl(d->base + DISPC_CTRL) | CTRL_RUN, d->base + DISPC_CTRL);
}

static void sprd_dispc_panel_off(struct sprd_dispc *d)
{
	int i;

	sprd_dispc_stop(d);
	sprd_dsi_cmd_mode(d);
	sprd_dsi_write(d, NT51017_SLEEP_IN, 0);
	msleep(160);
	for (i = d->panel_power->ndescs - 1; i >= 0; i--)
		gpiod_set_value_cansleep(d->panel_power->desc[i], 0);
	msleep(20);
	d->panel_off = true;
}

/* the bootloader's power-up: supplies, LP-11 for 40 ms, video */
static void sprd_dispc_panel_on(struct sprd_dispc *d)
{
	int i, id, status;

	for (i = 0; i < d->panel_power->ndescs; i++) {
		gpiod_set_value_cansleep(d->panel_power->desc[i], 1);
		if (!i)
			usleep_range(1000, 2000);
	}
	msleep(40);
	id = sprd_dsi_read(d, NT51017_ID_REG);
	status = sprd_dsi_read(d, NT51017_STATUS_REG);
	if (id != NT51017_ID)
		drm_warn(&d->drm, "panel ID %d after power-up, status %d\n",
			 id, status);
	else
		drm_dbg_kms(&d->drm, "panel on, status %#x\n", status);
	sprd_dsi_video_mode(d);
	sprd_dispc_run(d);
	d->panel_off = false;
}

/* runs before the planes (commit_tail_rpm), so vblank is on for the flip */
static void sprd_dispc_crtc_atomic_enable(struct drm_crtc *crtc,
					  struct drm_atomic_commit *state)
{
	struct sprd_dispc *d = to_sprd_dispc(crtc->dev);
	bool powered_up = d->panel_off;

	if (powered_up)
		sprd_dispc_panel_on(d);
	drm_crtc_vblank_on(crtc);
	/* two frames of video before the light, as the vendor waits */
	if (powered_up)
		msleep(43);
	backlight_enable(d->backlight);
}

static void sprd_dispc_crtc_atomic_disable(struct drm_crtc *crtc,
					   struct drm_atomic_commit *state)
{
	struct sprd_dispc *d = to_sprd_dispc(crtc->dev);

	backlight_disable(d->backlight);
	sprd_dispc_flip_sync(d, d->black_dma);
	drm_crtc_vblank_off(crtc);
	cancel_delayed_work_sync(&d->flip_timeout);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	if (panel_power_off && d->dsi && d->panel_power)
		sprd_dispc_panel_off(d);
}

/* the DPI block counts frames in hardware */
static u32 sprd_dispc_get_vblank_counter(struct drm_crtc *crtc)
{
	struct sprd_dispc *d = to_sprd_dispc(crtc->dev);

	return FIELD_GET(DPI_STS1_FRAME, readl(d->base + DISPC_DPI_STS1));
}

static int sprd_dispc_enable_vblank(struct drm_crtc *crtc)
{
	sprd_dispc_int_update(to_sprd_dispc(crtc->dev), 0, INT_VSYNC);
	return 0;
}

static void sprd_dispc_disable_vblank(struct drm_crtc *crtc)
{
	sprd_dispc_int_update(to_sprd_dispc(crtc->dev), INT_VSYNC, 0);
}

static enum drm_mode_status
sprd_dispc_crtc_mode_valid(struct drm_crtc *crtc,
			   const struct drm_display_mode *mode)
{
	struct sprd_dispc *d = to_sprd_dispc(crtc->dev);

	/* the mode is whatever the bootloader programmed */
	if (mode->hdisplay != d->mode.hdisplay ||
	    mode->vdisplay != d->mode.vdisplay)
		return MODE_BAD;
	return MODE_OK;
}

static const struct drm_crtc_helper_funcs sprd_dispc_crtc_helper_funcs = {
	.mode_valid = sprd_dispc_crtc_mode_valid,
	.atomic_check = sprd_dispc_crtc_atomic_check,
	.atomic_flush = sprd_dispc_crtc_atomic_flush,
	.atomic_enable = sprd_dispc_crtc_atomic_enable,
	.atomic_disable = sprd_dispc_crtc_atomic_disable,
};

static const struct drm_crtc_funcs sprd_dispc_crtc_funcs = {
	.atomic_create_state = drm_atomic_helper_crtc_create_state,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = sprd_dispc_enable_vblank,
	.disable_vblank = sprd_dispc_disable_vblank,
	.get_vblank_counter = sprd_dispc_get_vblank_counter,
};

static int sprd_dispc_connector_get_modes(struct drm_connector *connector)
{
	struct sprd_dispc *d = to_sprd_dispc(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, &d->mode);
}

static const struct drm_connector_helper_funcs sprd_dispc_connector_helper_funcs = {
	.get_modes = sprd_dispc_connector_get_modes,
};

static const struct drm_connector_funcs sprd_dispc_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_gem_object *
sprd_dispc_gem_create_object(struct drm_device *drm, size_t size)
{
	struct drm_gem_dma_object *obj;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return ERR_PTR(-ENOMEM);
	obj->map_noncoherent = true;
	return &obj->base;
}

static const struct drm_mode_config_helper_funcs sprd_dispc_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

static const struct drm_mode_config_funcs sprd_dispc_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(sprd_dispc_fops);

static const struct drm_driver sprd_dispc_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name = "sprd-dispc",
	.desc = "Spreadtrum SC7730/SC8830 display controller",
	.major = 1,
	.minor = 0,
	.fops = &sprd_dispc_fops,
	.gem_create_object = sprd_dispc_gem_create_object,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
};

static void __iomem *sprd_dispc_map(struct platform_device *pdev,
				    const char *name)
{
	struct resource *res;

	/* gate and clock registers are shared with other code: no request */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return NULL;
	return devm_ioremap(&pdev->dev, res->start, resource_size(res));
}

/*
 * Only take over a pipeline that runs as expected: reading a gated
 * DISPC can hang the bus, so look at the gates first.
 */
static int sprd_dispc_check_hw(struct platform_device *pdev,
			       struct sprd_dispc *d)
{
	struct device *dev = &pdev->dev;
	void __iomem *ahb = sprd_dispc_map(pdev, "ap-ahb");
	void __iomem *aon = sprd_dispc_map(pdev, "aon-apb");
	void __iomem *apb = sprd_dispc_map(pdev, "ap-apb");
	void __iomem *clk = sprd_dispc_map(pdev, "ap-clk");
	u32 ctrl, dpi, size, osd, osd_size, pitch, xy, h, v, cfg, div, mm;
	struct drm_display_mode *m = &d->mode;
	unsigned long rate;

	if (!ahb || !aon || !apb || !clk)
		return -ENOMEM;
	if (!(readl(ahb + AP_AHB_EB) & AP_AHB_DISPC_EB) ||
	    !(readl(aon + AON_APB_EB1) & AON_DISP_EMC_EB) ||
	    !(readl(apb + AP_APB_EB) & AP_APB_CKG_EB))
		return dev_err_probe(dev, -ENODEV, "DISPC is not clocked\n");

	ctrl = readl(d->base + DISPC_CTRL);
	dpi = readl(d->base + DISPC_DPI_CTRL);
	size = readl(d->base + DISPC_SIZE_XY);
	osd = readl(d->base + DISPC_OSD_CTRL);
	osd_size = readl(d->base + DISPC_OSD_SIZE_XY);
	pitch = readl(d->base + DISPC_OSD_PITCH);
	xy = readl(d->base + DISPC_OSD_DISP_XY);
	h = readl(d->base + DISPC_DPI_H_TIMING);
	v = readl(d->base + DISPC_DPI_V_TIMING);
	cfg = readl(clk + AP_CLK_DISPC_DPI_CFG);
	dev_info(dev,
		 "CTRL %08x DPI_CTRL %08x SIZE %08x OSD %08x/%08x/%08x/%08x/%08x timing %08x %08x DPI clock %08x\n",
		 ctrl, dpi, size, osd, readl(d->base + DISPC_OSD_BASE_ADDR),
		 osd_size, pitch, xy, h, v, cfg);

	if ((ctrl & (CTRL_EN | CTRL_IF | CTRL_RUN)) != (CTRL_EN | CTRL_RUN) ||
	    (dpi & DPI_SW_UPDATE_ONLY) ||
	    (osd & ~OSD_CTRL_RB_SWITCH) != OSD_CTRL_RGB8888 ||
	    osd_size != size || xy ||
	    pitch != FIELD_GET(SIZE_W, size))
		return dev_err_probe(dev, -ENODEV,
				     "unexpected DISPC state, leaving it alone\n");

	div = FIELD_GET(DPI_CLK_DIV, cfg);
	rate = sprd_dispc_dpi_parents[FIELD_GET(DPI_CLK_SEL, cfg)] / (div + 1);

	m->hdisplay = FIELD_GET(SIZE_W, size);
	m->hsync_start = m->hdisplay + FIELD_GET(DPI_FP, h);
	m->hsync_end = m->hsync_start + FIELD_GET(DPI_SYNC, h);
	m->htotal = m->hsync_end + FIELD_GET(DPI_BP, h);
	m->vdisplay = FIELD_GET(SIZE_H, size);
	m->vsync_start = m->vdisplay + FIELD_GET(DPI_FP, v);
	m->vsync_end = m->vsync_start + FIELD_GET(DPI_SYNC, v);
	m->vtotal = m->vsync_end + FIELD_GET(DPI_BP, v);
	m->clock = rate / 1000;
	m->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	if (!of_property_read_u32(dev->of_node, "width-mm", &mm))
		m->width_mm = mm;
	if (!of_property_read_u32(dev->of_node, "height-mm", &mm))
		m->height_mm = mm;
	drm_mode_set_name(m);
	d->rb_switch = osd & OSD_CTRL_RB_SWITCH;
	d->fb_size = m->hdisplay * m->vdisplay * 4;
	return 0;
}

static int sprd_dispc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_device *drm;
	struct sprd_dispc *d;
	int irq, ret;

	d = devm_drm_dev_alloc(dev, &sprd_dispc_driver, struct sprd_dispc, drm);
	if (IS_ERR(d))
		return PTR_ERR(d);
	drm = &d->drm;
	spin_lock_init(&d->int_lock);
	init_completion(&d->update_done);
	INIT_DELAYED_WORK(&d->flip_timeout, sprd_dispc_flip_timeout);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	d->base = sprd_dispc_map(pdev, "dispc");
	if (!d->base)
		return -ENOMEM;
	ret = sprd_dispc_check_hw(pdev, d);
	if (ret)
		return ret;

	d->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(d->backlight))
		return PTR_ERR(d->backlight);

	/* optional: switching the panel off needs the DSI host and supplies */
	d->dsi = sprd_dispc_map(pdev, "dsi");
	d->panel_power = devm_gpiod_get_array_optional(dev, "panel-power",
						       GPIOD_OUT_HIGH);
	if (IS_ERR(d->panel_power))
		return dev_err_probe(dev, PTR_ERR(d->panel_power),
				     "panel supplies\n");

	d->black = dmam_alloc_coherent(dev, d->fb_size, &d->black_dma,
				       GFP_KERNEL);
	if (!d->black)
		return -ENOMEM;
	d->pending_addr = d->black_dma;
	d->pending_pitch = d->mode.hdisplay;

	/* the bootloader leaves a stale UPDATE_DONE behind */
	writel(0, d->base + DISPC_INT_EN);
	writel(INT_ALL, d->base + DISPC_INT_CLR);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, sprd_dispc_irq, 0, dev_name(dev), d);
	if (ret)
		return ret;
	sprd_dispc_int_update(d, 0, INT_UPDATE_DONE | INT_ERR);

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = d->mode.hdisplay;
	drm->mode_config.max_width = d->mode.hdisplay;
	drm->mode_config.min_height = d->mode.vdisplay;
	drm->mode_config.max_height = d->mode.vdisplay;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &sprd_dispc_mode_config_funcs;
	drm->mode_config.helper_private = &sprd_dispc_mode_config_helpers;

	ret = drm_universal_plane_init(drm, &d->plane, 0, &sprd_dispc_plane_funcs,
				       sprd_dispc_formats,
				       ARRAY_SIZE(sprd_dispc_formats), NULL,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&d->plane, &sprd_dispc_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(&d->plane);

	ret = drm_crtc_init_with_planes(drm, &d->crtc, &d->plane, NULL,
					&sprd_dispc_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&d->crtc, &sprd_dispc_crtc_helper_funcs);

	ret = drmm_encoder_init(drm, &d->encoder, NULL, DRM_MODE_ENCODER_DSI, NULL);
	if (ret)
		return ret;
	d->encoder.possible_crtcs = drm_crtc_mask(&d->crtc);

	ret = drm_connector_init(drm, &d->connector, &sprd_dispc_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret)
		return ret;
	drm_connector_helper_add(&d->connector, &sprd_dispc_connector_helper_funcs);
	d->connector.display_info.width_mm = d->mode.width_mm;
	d->connector.display_info.height_mm = d->mode.height_mm;
	ret = drm_connector_attach_encoder(&d->connector, &d->encoder);
	if (ret)
		return ret;

	drm->max_vblank_count = FIELD_MAX(DPI_STS1_FRAME);
	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;
	drm_mode_config_reset(drm);

	/* simpledrm scans out through the same pipeline: send it away */
	ret = aperture_remove_conflicting_devices(readl(d->base + DISPC_OSD_BASE_ADDR),
						  d->fb_size, sprd_dispc_driver.name);
	if (ret)
		return ret;

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, drm);
	drm_info(drm, "taking over %ux%u@%u from the bootloader\n",
		 d->mode.hdisplay, d->mode.vdisplay, drm_mode_vrefresh(&d->mode));
	drm_client_setup(drm, NULL);
	return 0;
}

static void sprd_dispc_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct sprd_dispc *d = to_sprd_dispc(drm);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
	cancel_delayed_work_sync(&d->flip_timeout);
	writel(0, d->base + DISPC_INT_EN);
}

static void sprd_dispc_shutdown(struct platform_device *pdev)
{
	drm_atomic_helper_shutdown(platform_get_drvdata(pdev));
}

static int sprd_dispc_suspend(struct device *dev)
{
	return drm_mode_config_helper_suspend(dev_get_drvdata(dev));
}

static int sprd_dispc_resume(struct device *dev)
{
	return drm_mode_config_helper_resume(dev_get_drvdata(dev));
}

static DEFINE_SIMPLE_DEV_PM_OPS(sprd_dispc_pm_ops, sprd_dispc_suspend,
				sprd_dispc_resume);

static const struct of_device_id sprd_dispc_of_match[] = {
	{ .compatible = "sprd,sc7730-dispc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_dispc_of_match);

static struct platform_driver sprd_dispc_platform_driver = {
	.probe = sprd_dispc_probe,
	.remove = sprd_dispc_remove,
	.shutdown = sprd_dispc_shutdown,
	.driver = {
		.name = "sprd-dispc",
		.of_match_table = sprd_dispc_of_match,
		.pm = pm_sleep_ptr(&sprd_dispc_pm_ops),
	},
};
module_platform_driver(sprd_dispc_platform_driver);

MODULE_DESCRIPTION("Spreadtrum SC7730/SC8830 display controller takeover driver");
MODULE_LICENSE("GPL");
