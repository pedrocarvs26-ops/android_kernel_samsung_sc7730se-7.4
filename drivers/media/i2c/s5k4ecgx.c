// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K4ECGX, 5 MP YUV camera sensor with its own ISP (rear camera
 * of the Samsung Galaxy Tab E 9.6), MIPI CSI-2 with two lanes.
 *
 * Like the SR200PC20M, the sensor is set up entirely from register tables
 * of the stock Android camera HAL, loaded as firmware: s5k4ecgx/init.bin
 * and one file per preview size (tools/camtables.py in the gtelwifi
 * bring-up extracts them). Files are little-endian {u16 reg, u16 val}
 * pairs; 0xffff NNNN is a delay of NNNN ms. Most entries go through the
 * indirect interface into the ISP's RAM: 0x0028/0x002a set the address,
 * each write to 0x0f12 stores a word and advances it by two.
 *
 * The init table ends by enabling the preview. Those two writes are
 * skipped here, so that the preview size is set first and the stream
 * starts in enable_streams; until then the lanes idle in LP-11. Stream
 * on and off use REG_G_ENABLE_PREV and REG_G_ENABLE_PREV_CHG as the
 * former mainline driver did.
 *
 * Still captures (module parameter still_modes, experimental): the ISP
 * has a capture configuration next to the preview one. A capture size
 * table of the HAL sets its input window and output size, the switch is
 * REG_TC_GP_EnableCapture with its "changed" flag. As in the stock HAL,
 * AE and AWB (and one autofocus search) settle in a preview first. The
 * switch happens in pre_streamon, before the receiver starts, so that
 * the receiver only ever sees capture frames; the lanes are then in HS
 * when it starts, as they always were with the vendor stack. The output
 * stays YUV 4:2:2 at 344 Mbit/s per lane (both system clocks of the init
 * table have the same output rate).
 *
 * Autofocus, following the HAL: AUTO_FOCUS_RANGE writes its normal or
 * macro lens table (s5k4ecgx/af-normal.bin, af-macro.bin, optional),
 * AUTO_FOCUS_START sends a single search (REG_TC_AF_AfCmd 5) and polls
 * the coarse search status, then the fine search flag. The S5K HAL has no
 * continuous AF. libcamera does not use any of these controls.
 *
 * The capture switch and AF command/status words are the sensor
 * firmware's interface variables (also in Samsung's GPL S5K4ECGX
 * drivers); everything with tuning or lens data stays in firmware files.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* direct registers */
#define S5K_REG_AHB_PAGE	CCI_REG16(0xfcfc)
#define  S5K_AHB_PAGE		0xd000
#define S5K_REG_WR_ADDRH	CCI_REG16(0x0028)
#define S5K_REG_WR_ADDRL	CCI_REG16(0x002a)
#define S5K_REG_RD_ADDRH	CCI_REG16(0x002c)
#define S5K_REG_RD_ADDRL	CCI_REG16(0x002e)
#define S5K_REG_DATA		CCI_REG16(0x0f12)
/* ISP RAM */
#define S5K_FW_VERSION		0x700001a4
#define  S5K_ID			0x4ec0
#define S5K_FW_REVISION		0x700001a6
#define S5K_G_ENABLE_PREV	0x7000023e
#define S5K_G_ENABLE_PREV_CHG	0x70000240
/* still capture: as the preview enable, plus a configuration sync */
#define S5K_G_ENABLE_CAP	0x70000242	/* REG_TC_GP_EnableCapture */
#define S5K_G_ENABLE_CAP_CHG	0x70000244
#define S5K_G_NEW_CFG_SYNC	0x7000024e
/*
 * User adjustments on top of the tuning, signed, 0 = unchanged. Addresses
 * as the stock HAL's contrast/saturation tables write them (and measured:
 * 0x230 moves the luma, 0x234 the chroma); the old mainline driver's
 * 0x22c/0x22e/0x230 are one word off for this firmware.
 */
#define S5K_USER_BRIGHTNESS	0x70000230
#define S5K_USER_CONTRAST	0x70000232
#define S5K_USER_SATURATION	0x70000234
#define S5K_SATURATION_DEF	0
/*
 * Preview config 0, longest frame time in 1/10 ms. The init allows 100 ms
 * (10 fps), which the exposure uses up indoors; 66.6 ms is the HAL's
 * 15 fps value. The shortest frame time stays at 33.3 ms (30 fps).
 */
#define S5K_PCFG0_MAX_FR_TIME	0x700002c2
#define  S5K_MAX_FR_TIME_15FPS	666
/* autofocus */
#define S5K_AF_CMD		0x7000028c	/* REG_TC_AF_AfCmd */
#define  S5K_AF_CMD_LENS	4		/* lens to S5K_AF_CMD_PARAM */
#define  S5K_AF_CMD_SINGLE	5		/* one search */
#define S5K_AF_CMD_PARAM	0x7000028e
#define S5K_AF_SEARCH_STATUS	0x70002eee	/* 1 searching, 2 found */
#define S5K_AF_FINE_STATUS	0x70002207	/* 15:8 non-zero while busy */
#define S5K_AE_STABLE		0x70002c74	/* 1 when AE has settled */

#define S5K_MCLK_HZ		24000000
#define S5K_LANES		2

#define S5K_AF_POLL_MS		50
#define S5K_AF_SEARCH_POLLS	100	/* the HAL gives up after ~5 s */
#define S5K_AF_FINE_POLLS	60

struct s5k_mode {
	unsigned int width, height;
	const char *fw;
	bool still;		/* a capture configuration */
};

/* 344 Mbit/s per lane, DDR, set up by the init table for all sizes */
static const s64 s5k_link_freqs[] = { 172000000 };

/*
 * Preview sizes only up to 800x480: camera applications such as GNOME
 * Snapshot pick the size closest to 1280x720 and render it on the CPU
 * (no GLES 3 on the Mali-400), which keeps four Cortex-A7 at about 70 %
 * for 1280x720 at 11 fps. The larger HAL preview tables (960x960,
 * 1024x768, 1280x720, 1280x768) come back with a still capture path.
 */
static const struct s5k_mode s5k_modes[] = {
	{ 640, 480, "s5k4ecgx/640x480.bin" },
	{ 800, 480, "s5k4ecgx/800x480.bin" },
	/* capture configurations, only with still_modes */
	{ 1600, 1200, "s5k4ecgx/1600x1200.bin", true },
	{ 2048, 1536, "s5k4ecgx/2048x1536.bin", true },
	{ 2560, 1920, "s5k4ecgx/2560x1920.bin", true },
	{ 2576, 1932, "s5k4ecgx/2576x1932.bin", true },
};

#define S5K_NUM_PREVIEW_MODES	2
/* the preview in which AE/AWB/AF settle before a still */
#define S5K_STILL_PREVIEW	(&s5k_modes[0])

static bool still_modes;
module_param(still_modes, bool, 0644);
MODULE_PARM_DESC(still_modes, "Offer the still sizes up to 2576x1932 (experimental)");

static unsigned int still_settle_ms = 1000;
module_param(still_settle_ms, uint, 0644);
MODULE_PARM_DESC(still_settle_ms,
		 "Preview time for AE/AWB before a still, 0 = switch right after the init");

static bool still_af = true;
module_param(still_af, bool, 0644);
MODULE_PARM_DESC(still_af, "One autofocus search before a still (needs still_settle_ms)");

static unsigned int still_skip = 1;
module_param(still_skip, uint, 0644);
MODULE_PARM_DESC(still_skip, "Frames the receiver drops after the capture switch (HAL: 1)");

static unsigned int preview_af_ms;
module_param(preview_af_ms, uint, 0644);
MODULE_PARM_DESC(preview_af_ms, "One autofocus search this long after a preview starts, 0 = off");

static const char * const s5k_supplies[] = { "dovdd", "avdd", "dvdd" };

enum { S5K_AF_IDLE, S5K_AF_START, S5K_AF_SEARCH, S5K_AF_FINE };

/* s5k_write_table() flags */
#define S5K_TAB_SKIP_ENABLE	BIT(0)	/* leave out the preview enable */
#define S5K_TAB_OPTIONAL	BIT(1)	/* -ENOENT without an error if missing */

struct s5k4ecgx {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdown_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(s5k_supplies)];
	struct regulator *afvdd;	/* AF actuator, optional */
	struct v4l2_ctrl_handler ctrls;
	const struct s5k_mode *mode;
	const struct s5k_mode *loaded;	/* tables in the ISP, NULL after power off */
	const struct s5k_mode *still;	/* switched to this capture configuration */
	bool powered;			/* by pre_streamon */

	/* under the control handler lock, as all access to the ISP */
	bool af_ok;			/* a preview is running */
	int af_range;			/* lens table in the ISP: 0 normal, 1 macro */
	unsigned int af_phase, af_polls, af_retries;
	u32 af_status;			/* V4L2_AUTO_FOCUS_STATUS_* */
	unsigned long af_started;
	struct delayed_work af_work;
};

static inline struct s5k4ecgx *to_s5k(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k4ecgx, sd);
}

static unsigned int s5k_num_modes(void)
{
	return still_modes ? ARRAY_SIZE(s5k_modes) : S5K_NUM_PREVIEW_MODES;
}

/* one word into the ISP's RAM */
static int s5k_write(struct s5k4ecgx *s5k, u32 addr, u16 val)
{
	int ret = 0;

	cci_write(s5k->regmap, S5K_REG_WR_ADDRH, addr >> 16, &ret);
	cci_write(s5k->regmap, S5K_REG_WR_ADDRL, addr & 0xffff, &ret);
	cci_write(s5k->regmap, S5K_REG_DATA, val, &ret);
	return ret;
}

static int s5k_read(struct s5k4ecgx *s5k, u32 addr, u64 *val)
{
	int ret = 0;

	cci_write(s5k->regmap, S5K_REG_AHB_PAGE, S5K_AHB_PAGE, &ret);
	cci_write(s5k->regmap, S5K_REG_RD_ADDRH, addr >> 16, &ret);
	cci_write(s5k->regmap, S5K_REG_RD_ADDRL, addr & 0xffff, &ret);
	cci_read(s5k->regmap, S5K_REG_DATA, val, &ret);
	return ret;
}

/* a few words, after selecting the ISP's AHB page */
static int s5k_write_words(struct s5k4ecgx *s5k, const u32 *addr,
			   const u16 *val, unsigned int n)
{
	unsigned int i;
	int ret = 0;

	cci_write(s5k->regmap, S5K_REG_AHB_PAGE, S5K_AHB_PAGE, &ret);
	for (i = 0; i < n && !ret; i++)
		ret = s5k_write(s5k, addr[i], val[i]);
	return ret;
}

/*
 * Write a HAL table. With S5K_TAB_SKIP_ENABLE, data words that would land
 * on REG_G_ENABLE_PREV or REG_G_ENABLE_PREV_CHG are left out.
 */
static int s5k_write_table(struct s5k4ecgx *s5k, const char *name,
			   unsigned int flags)
{
	const struct firmware *fw;
	const __le16 *p;
	u32 addr = 0;
	size_t i, n;
	int ret;

	if (flags & S5K_TAB_OPTIONAL) {
		ret = firmware_request_nowarn(&fw, name, s5k->dev);
		if (ret)
			return -ENOENT;
	} else {
		ret = request_firmware(&fw, name, s5k->dev);
		if (ret) {
			dev_err(s5k->dev,
				"%s missing (extract it from the stock HAL)\n",
				name);
			return ret;
		}
	}
	if (!fw->size || fw->size % 4) {
		dev_err(s5k->dev, "%s: bad size %zu\n", name, fw->size);
		release_firmware(fw);
		return -EINVAL;
	}

	p = (const __le16 *)fw->data;
	n = fw->size / 4;
	for (i = 0; i < n && !ret; i++) {
		u16 reg = le16_to_cpu(p[2 * i]), val = le16_to_cpu(p[2 * i + 1]);

		if (reg == 0xffff && val != 0xffff) {
			msleep(val);
			continue;
		}
		if (reg == 0x0028)
			addr = (addr & 0xffff) | val << 16;
		else if (reg == 0x002a)
			addr = (addr & 0xffff0000) | val;
		if (reg == 0x0f12) {
			bool skip = (flags & S5K_TAB_SKIP_ENABLE) &&
				    (addr == S5K_G_ENABLE_PREV ||
				     addr == S5K_G_ENABLE_PREV_CHG);

			addr += 2;
			if (skip) {
				/* move the pointer past the skipped word */
				cci_write(s5k->regmap, S5K_REG_WR_ADDRL,
					  addr & 0xffff, &ret);
				continue;
			}
		}
		cci_write(s5k->regmap, CCI_REG16(reg), val, &ret);
	}
	release_firmware(fw);
	if (ret)
		dev_err(s5k->dev, "%s: entry %zu failed: %d\n", name, i - 1, ret);
	return ret;
}

static int s5k_preview(struct s5k4ecgx *s5k, bool on)
{
	static const u32 addr[] = { S5K_G_ENABLE_PREV, S5K_G_ENABLE_PREV_CHG };
	const u16 val[] = { on, 1 };

	return s5k_write_words(s5k, addr, val, ARRAY_SIZE(addr));
}

/* the HAL's Capture_Start and Preview_Return */
static int s5k_capture(struct s5k4ecgx *s5k, bool on)
{
	static const u32 addr[] = {
		S5K_G_ENABLE_CAP, S5K_G_NEW_CFG_SYNC, S5K_G_ENABLE_CAP_CHG,
	};
	const u16 val[] = { on, 1, 1 };

	return s5k_write_words(s5k, addr, val, ARRAY_SIZE(addr));
}

/* --- autofocus --------------------------------------------------------- */

static int s5k_af_lens_home(struct s5k4ecgx *s5k)
{
	/* the HAL's AF_off: lens to position 0 of the current range */
	static const u32 addr[] = { S5K_AF_CMD_PARAM, S5K_AF_CMD };
	static const u16 val[] = { 0, S5K_AF_CMD_LENS };

	return s5k_write_words(s5k, addr, val, ARRAY_SIZE(addr));
}

static void s5k_af_done(struct s5k4ecgx *s5k, u32 status)
{
	s5k->af_status = status;
	s5k->af_phase = S5K_AF_IDLE;
	dev_info(s5k->dev, "autofocus %s after %u ms\n",
		 status == V4L2_AUTO_FOCUS_STATUS_REACHED ? "reached" : "failed",
		 jiffies_to_msecs(jiffies - s5k->af_started));
}

/*
 * One step of the HAL's single AF (_s5k4ec_AutoFocusDo): start, poll the
 * coarse search until it reports "found", then the fine search until its
 * busy byte clears. Returns the delay to the next step in ms, 0 when done.
 */
static int s5k_af_step(struct s5k4ecgx *s5k)
{
	static const u32 cmd = S5K_AF_CMD;
	static const u16 single = S5K_AF_CMD_SINGLE;
	u64 val = 0;
	int ret;

	switch (s5k->af_phase) {
	case S5K_AF_START:
		ret = s5k_write_words(s5k, &cmd, &single, 1);
		if (ret)
			break;
		s5k->af_phase = S5K_AF_SEARCH;
		s5k->af_polls = 0;
		return 100;

	case S5K_AF_SEARCH:
		ret = s5k_read(s5k, S5K_AF_SEARCH_STATUS, &val);
		if (ret)
			break;
		if (val == 2) {
			s5k->af_phase = S5K_AF_FINE;
			s5k->af_polls = 0;
			return S5K_AF_POLL_MS;
		}
		if (val == 0 || val == 8) {
			/* idle or cancelled: the HAL sends the start again */
			if (!s5k->af_retries++) {
				s5k->af_phase = S5K_AF_START;
				return 1;
			}
		}
		if (val != 1 || ++s5k->af_polls >= S5K_AF_SEARCH_POLLS) {
			dev_dbg(s5k->dev, "AF search status %llu\n", val);
			s5k_af_lens_home(s5k);
			s5k_af_done(s5k, V4L2_AUTO_FOCUS_STATUS_FAILED);
			return 0;
		}
		return S5K_AF_POLL_MS;

	case S5K_AF_FINE:
		ret = s5k_read(s5k, S5K_AF_FINE_STATUS, &val);
		if (ret)
			break;
		/* the HAL also takes a timeout here as success */
		if (!(val & 0xff00) || ++s5k->af_polls >= S5K_AF_FINE_POLLS) {
			s5k_af_done(s5k, V4L2_AUTO_FOCUS_STATUS_REACHED);
			return 0;
		}
		return S5K_AF_POLL_MS;

	default:
		return 0;
	}

	dev_err(s5k->dev, "autofocus: %d\n", ret);
	s5k_af_done(s5k, V4L2_AUTO_FOCUS_STATUS_FAILED);
	return ret;
}

static void s5k_af_work(struct work_struct *work)
{
	struct s5k4ecgx *s5k = container_of(to_delayed_work(work),
					    struct s5k4ecgx, af_work);
	int next;

	mutex_lock(s5k->ctrls.lock);
	/* stopped or streamoff in the meantime */
	if (s5k->af_ok && s5k->af_phase != S5K_AF_IDLE) {
		next = s5k_af_step(s5k);
		if (next > 0)
			schedule_delayed_work(&s5k->af_work,
					      msecs_to_jiffies(next));
	}
	mutex_unlock(s5k->ctrls.lock);
}

static void s5k_af_begin(struct s5k4ecgx *s5k, unsigned int delay_ms)
{
	s5k->af_status = V4L2_AUTO_FOCUS_STATUS_BUSY;
	s5k->af_phase = S5K_AF_START;
	s5k->af_retries = 0;
	s5k->af_started = jiffies + msecs_to_jiffies(delay_ms);
	mod_delayed_work(system_percpu_wq, &s5k->af_work,
			 msecs_to_jiffies(delay_ms));
}

/* the same, synchronously, under the control lock */
static void s5k_af_sync(struct s5k4ecgx *s5k)
{
	int next;

	s5k->af_status = V4L2_AUTO_FOCUS_STATUS_BUSY;
	s5k->af_phase = S5K_AF_START;
	s5k->af_retries = 0;
	s5k->af_started = jiffies;
	while ((next = s5k_af_step(s5k)) > 0)
		msleep(next);
}

static int s5k_af_stop(struct s5k4ecgx *s5k)
{
	cancel_delayed_work(&s5k->af_work);
	s5k->af_phase = S5K_AF_IDLE;
	s5k->af_status = V4L2_AUTO_FOCUS_STATUS_IDLE;
	return s5k->af_ok ? s5k_af_lens_home(s5k) : 0;
}

/* the HAL's AF_normal_mode_1..3 / AF_macro_mode_1..3, 100 ms apart */
static int s5k_af_range(struct s5k4ecgx *s5k, int range)
{
	int macro = range == V4L2_AUTO_FOCUS_RANGE_MACRO;
	const char *name = macro ? "s5k4ecgx/af-macro.bin" :
				   "s5k4ecgx/af-normal.bin";
	int ret;

	if (!s5k->loaded || s5k->af_range == macro)
		return 0;
	ret = s5k_write_table(s5k, name, S5K_TAB_OPTIONAL);
	if (ret == -ENOENT) {
		dev_info_once(s5k->dev, "%s missing, AF range unchanged\n",
			      name);
		return 0;
	}
	if (!ret)
		s5k->af_range = macro;
	return ret;
}

/* --- power and streaming ----------------------------------------------- */

static int s5k_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4ecgx *s5k = to_s5k(sd);
	int ret;

	gpiod_set_value_cansleep(s5k->reset_gpio, 1);
	gpiod_set_value_cansleep(s5k->powerdown_gpio, 1);
	/* vendor order: AF actuator, I/O, analog, MCLK, external 1.2 V core */
	if (s5k->afvdd) {
		ret = regulator_enable(s5k->afvdd);
		if (ret)
			return ret;
		usleep_range(1000, 2000);
	}
	ret = regulator_enable(s5k->supplies[0].consumer);
	if (ret)
		goto err_afvdd;
	ret = regulator_enable(s5k->supplies[1].consumer);
	if (ret)
		goto err_dovdd;
	ret = clk_prepare_enable(s5k->mclk);
	if (ret)
		goto err_avdd;
	usleep_range(1000, 2000);
	ret = regulator_enable(s5k->supplies[2].consumer);
	if (ret)
		goto err_clk;
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(s5k->powerdown_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(s5k->reset_gpio, 0);
	msleep(10);
	return 0;

err_clk:
	clk_disable_unprepare(s5k->mclk);
err_avdd:
	regulator_disable(s5k->supplies[1].consumer);
err_dovdd:
	regulator_disable(s5k->supplies[0].consumer);
err_afvdd:
	if (s5k->afvdd)
		regulator_disable(s5k->afvdd);
	return ret;
}

static int s5k_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4ecgx *s5k = to_s5k(sd);

	gpiod_set_value_cansleep(s5k->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(s5k->powerdown_gpio, 1);
	regulator_disable(s5k->supplies[2].consumer);
	clk_disable_unprepare(s5k->mclk);
	regulator_disable(s5k->supplies[1].consumer);
	regulator_disable(s5k->supplies[0].consumer);
	if (s5k->afvdd)
		regulator_disable(s5k->afvdd);
	s5k->loaded = NULL;
	s5k->still = NULL;
	return 0;
}

static void s5k_fill_fmt(const struct s5k_mode *mode,
			 struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_SRGB;
}

static int s5k_init_state(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state)
{
	s5k_fill_fmt(&s5k_modes[0], v4l2_subdev_state_get_format(state, 0));
	return 0;
}

static int s5k_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_UYVY8_1X16;
	return 0;
}

static int s5k_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_UYVY8_1X16 ||
	    fse->index >= s5k_num_modes())
		return -EINVAL;
	fse->min_width = fse->max_width = s5k_modes[fse->index].width;
	fse->min_height = fse->max_height = s5k_modes[fse->index].height;
	return 0;
}

static int s5k_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *format)
{
	struct s5k4ecgx *s5k = to_s5k(sd);
	const struct s5k_mode *mode;

	mode = v4l2_find_nearest_size(s5k_modes, s5k_num_modes(),
				      width, height, format->format.width,
				      format->format.height);
	s5k_fill_fmt(mode, &format->format);
	*v4l2_subdev_state_get_format(state, 0) = format->format;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		s5k->mode = mode;
	return 0;
}

static int s5k_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_selection *sel)
{
	struct v4l2_mbus_framefmt *fmt = v4l2_subdev_state_get_format(state, 0);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = sel->r.top = 0;
		sel->r.width = fmt->width;
		sel->r.height = fmt->height;
		return 0;
	}
	return -EINVAL;
}

/* init (resets the ISP) and a size table, then the user controls again */
static int s5k_load(struct s5k4ecgx *s5k, const struct s5k_mode *mode)
{
	int ret;

	if (s5k->loaded == mode)
		return 0;
	s5k->loaded = NULL;
	s5k->still = NULL;
	s5k->af_range = -1;
	ret = s5k_write_table(s5k, "s5k4ecgx/init.bin", S5K_TAB_SKIP_ENABLE);
	/* before the size table, which ends with the config sync */
	if (!ret)
		cci_write(s5k->regmap, S5K_REG_AHB_PAGE, S5K_AHB_PAGE, &ret);
	if (!ret)
		ret = s5k_write(s5k, S5K_PCFG0_MAX_FR_TIME, S5K_MAX_FR_TIME_15FPS);
	if (!ret)
		ret = s5k_write_table(s5k, mode->fw, S5K_TAB_SKIP_ENABLE);
	if (ret)
		return ret;
	s5k->loaded = mode;
	/* the init resets the adjustments too */
	return __v4l2_ctrl_handler_setup(&s5k->ctrls);
}

/*
 * Switch to the capture configuration of the current still mode, under
 * the control lock and powered. Like the stock HAL (before_snapshot, then
 * Capture_Start in stream_on), the capture size table is written while
 * a preview runs, so that the capture takes over its exposure.
 */
static int s5k_still_start(struct s5k4ecgx *s5k)
{
	const struct s5k_mode *mode = s5k->mode;
	unsigned long timeout;
	u64 stable = 0;
	int ret;

	if (s5k->still == mode)
		return 0;
	s5k->loaded = NULL;	/* whatever ran before, start from the init */
	ret = s5k_load(s5k, S5K_STILL_PREVIEW);
	if (ret)
		return ret;

	if (still_settle_ms) {
		ret = s5k_preview(s5k, true);
		if (ret)
			return ret;
		msleep(still_settle_ms);
		timeout = jiffies + msecs_to_jiffies(1000);
		do {
			ret = s5k_read(s5k, S5K_AE_STABLE, &stable);
			if (ret || stable == 1)
				break;
			msleep(20);
		} while (time_before(jiffies, timeout));
		if (ret)
			return ret;
		if (still_af)
			s5k_af_sync(s5k);
	}

	ret = s5k_write_table(s5k, mode->fw, 0);
	if (!ret)
		ret = s5k_capture(s5k, true);
	if (ret)
		return ret;
	s5k->still = mode;
	/* test build: say what the capture is based on */
	dev_info(s5k->dev, "still %ux%u: settle %u ms, AE %s, AF %s\n",
		 mode->width, mode->height, still_settle_ms,
		 !still_settle_ms ? "-" : stable == 1 ? "stable" : "not stable",
		 !still_settle_ms || !still_af ? "off" :
		 s5k->af_status == V4L2_AUTO_FOCUS_STATUS_REACHED ? "reached" :
								    "failed");
	/* the switch takes effect at a frame boundary */
	msleep(100);
	return 0;
}

static int s5k_still_stop(struct s5k4ecgx *s5k)
{
	int ret;

	if (!s5k->still)
		return 0;
	s5k->still = NULL;
	s5k->loaded = NULL;	/* the next stream starts from the init */
	ret = s5k_capture(s5k, false);
	if (!ret)
		ret = s5k_preview(s5k, false);
	return ret;
}

static int s5k_get_power(struct s5k4ecgx *s5k)
{
	int ret;

	if (s5k->powered)
		return 0;
	ret = pm_runtime_resume_and_get(s5k->dev);
	if (ret)
		return ret;
	s5k->powered = true;
	return 0;
}

static void s5k_put_power(struct s5k4ecgx *s5k)
{
	if (s5k->powered) {
		s5k->powered = false;
		pm_runtime_put_autosuspend(s5k->dev);
	}
}

/*
 * Preview modes: powered, preview off, the lanes idle in LP-11. Still
 * modes: already switched to capture and streaming.
 */
static int s5k_pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct s5k4ecgx *s5k = to_s5k(sd);
	int ret;

	ret = s5k_get_power(s5k);
	if (ret)
		return ret;
	mutex_lock(s5k->ctrls.lock);
	ret = s5k->mode->still ? s5k_still_start(s5k) : 0;
	mutex_unlock(s5k->ctrls.lock);
	if (ret)
		s5k_put_power(s5k);
	return ret;
}

static int s5k_post_streamoff(struct v4l2_subdev *sd)
{
	struct s5k4ecgx *s5k = to_s5k(sd);

	/* a still started by pre_streamon that never got enable_streams */
	mutex_lock(s5k->ctrls.lock);
	if (s5k->still)
		s5k_still_stop(s5k);
	mutex_unlock(s5k->ctrls.lock);
	s5k_put_power(s5k);
	return 0;
}

static int s5k_enable_streams(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state, u32 pad,
			      u64 streams_mask)
{
	struct s5k4ecgx *s5k = to_s5k(sd);
	int ret;

	ret = s5k_get_power(s5k);
	if (ret)
		return ret;
	/* normally streaming since pre_streamon */
	if (s5k->mode->still)
		return s5k_still_start(s5k);

	/* the init resets the ISP; the size tables differ in more than size */
	ret = s5k_load(s5k, s5k->mode);
	if (!ret)
		ret = s5k_preview(s5k, true);
	if (ret)
		return ret;
	s5k->af_ok = true;
	if (preview_af_ms)
		s5k_af_begin(s5k, preview_af_ms);
	return 0;
}

static int s5k_disable_streams(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state, u32 pad,
			       u64 streams_mask)
{
	struct s5k4ecgx *s5k = to_s5k(sd);

	/* the work sees af_ok cleared under the lock and does nothing */
	s5k->af_ok = false;
	s5k->af_phase = S5K_AF_IDLE;
	s5k->af_status = V4L2_AUTO_FOCUS_STATUS_IDLE;
	cancel_delayed_work(&s5k->af_work);
	if (s5k->still)
		return s5k_still_stop(s5k);
	return s5k_preview(s5k, false);
}

/* HAL: two frames after a preview start, one after the capture switch */
static int s5k_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	*frames = to_s5k(sd)->mode->still ? still_skip : 2;
	return 0;
}

static int s5k_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4ecgx *s5k = container_of(ctrl->handler, struct s5k4ecgx,
					    ctrls);
	int ret = 0;
	u32 addr;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		addr = S5K_USER_BRIGHTNESS;
		break;
	case V4L2_CID_CONTRAST:
		addr = S5K_USER_CONTRAST;
		break;
	case V4L2_CID_SATURATION:
		addr = S5K_USER_SATURATION;
		break;
	case V4L2_CID_AUTO_FOCUS_RANGE:
		return s5k_af_range(s5k, ctrl->val);
	case V4L2_CID_AUTO_FOCUS_START:
		/* the search works on the preview statistics */
		if (!s5k->af_ok)
			return -EBUSY;
		s5k_af_begin(s5k, 0);
		return 0;
	case V4L2_CID_AUTO_FOCUS_STOP:
		return s5k_af_stop(s5k);
	default:
		/* link frequency and pixel rate follow the mode */
		return 0;
	}

	/* without the tables in the ISP: written after loading them */
	if (!s5k->loaded)
		return 0;

	cci_write(s5k->regmap, S5K_REG_AHB_PAGE, S5K_AHB_PAGE, &ret);
	if (!ret)
		ret = s5k_write(s5k, addr, ctrl->val);
	return ret;
}

static int s5k_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4ecgx *s5k = container_of(ctrl->handler, struct s5k4ecgx,
					    ctrls);

	if (ctrl->id == V4L2_CID_AUTO_FOCUS_STATUS)
		ctrl->val = s5k->af_status;
	return 0;
}

static const struct v4l2_ctrl_ops s5k_ctrl_ops = {
	.g_volatile_ctrl = s5k_g_volatile_ctrl,
	.s_ctrl = s5k_s_ctrl,
};

static const struct v4l2_subdev_video_ops s5k_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
	.pre_streamon = s5k_pre_streamon,
	.post_streamoff = s5k_post_streamoff,
};

static const struct v4l2_subdev_sensor_ops s5k_sensor_ops = {
	.g_skip_frames = s5k_g_skip_frames,
};

static const struct v4l2_subdev_pad_ops s5k_pad_ops = {
	.enum_mbus_code = s5k_enum_mbus_code,
	.enum_frame_size = s5k_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k_set_fmt,
	.get_selection = s5k_get_selection,
	.enable_streams = s5k_enable_streams,
	.disable_streams = s5k_disable_streams,
};

static const struct v4l2_subdev_ops s5k_subdev_ops = {
	.video = &s5k_video_ops,
	.sensor = &s5k_sensor_ops,
	.pad = &s5k_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k_internal_ops = {
	.init_state = s5k_init_state,
};

static int s5k_init_controls(struct s5k4ecgx *s5k)
{
	struct v4l2_ctrl_handler *hdl = &s5k->ctrls;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(hdl, 11);
	v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_BRIGHTNESS,
			  -127, 127, 1, 0);
	v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_CONTRAST,
			  -127, 127, 1, 0);
	v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_SATURATION,
			  -127, 127, 1, S5K_SATURATION_DEF);
	v4l2_ctrl_new_std_menu(hdl, &s5k_ctrl_ops, V4L2_CID_AUTO_FOCUS_RANGE,
			       V4L2_AUTO_FOCUS_RANGE_MACRO, 0,
			       V4L2_AUTO_FOCUS_RANGE_AUTO);
	v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_AUTO_FOCUS_START,
			  0, 0, 0, 0);
	v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_AUTO_FOCUS_STOP,
			  0, 0, 0, 0);
	/* read-only and volatile by definition */
	v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_AUTO_FOCUS_STATUS,
			  0, V4L2_AUTO_FOCUS_STATUS_BUSY |
			  V4L2_AUTO_FOCUS_STATUS_REACHED |
			  V4L2_AUTO_FOCUS_STATUS_FAILED, 0, 0);
	ctrl = v4l2_ctrl_new_int_menu(hdl, &s5k_ctrl_ops, V4L2_CID_LINK_FREQ,
				      0, 0, s5k_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	/* 16 bits per pixel on two DDR lanes */
	ctrl = v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_PIXEL_RATE,
				 1, INT_MAX, 1,
				 s5k_link_freqs[0] * 2 * S5K_LANES / 16);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ret = v4l2_fwnode_device_parse(s5k->dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(hdl, &s5k_ctrl_ops, &props);
	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}
	s5k->sd.ctrl_handler = hdl;
	return 0;
}

static int s5k_check_endpoint(struct s5k4ecgx *s5k)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *fw;
	int ret;

	fw = fwnode_graph_get_next_endpoint(dev_fwnode(s5k->dev), NULL);
	if (!fw)
		return dev_err_probe(s5k->dev, -EINVAL, "no endpoint\n");
	ret = v4l2_fwnode_endpoint_alloc_parse(fw, &ep);
	fwnode_handle_put(fw);
	if (ret)
		return dev_err_probe(s5k->dev, ret, "bad endpoint\n");
	if (ep.bus.mipi_csi2.num_data_lanes != S5K_LANES)
		ret = dev_err_probe(s5k->dev, -EINVAL, "two data lanes only\n");
	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

static int s5k_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k4ecgx *s5k;
	u64 id = 0, rev = 0;
	int i, ret;

	s5k = devm_kzalloc(dev, sizeof(*s5k), GFP_KERNEL);
	if (!s5k)
		return -ENOMEM;
	s5k->dev = dev;
	s5k->mode = &s5k_modes[0];
	s5k->af_range = -1;
	INIT_DELAYED_WORK(&s5k->af_work, s5k_af_work);
	v4l2_i2c_subdev_init(&s5k->sd, client, &s5k_subdev_ops);
	s5k->sd.internal_ops = &s5k_internal_ops;

	ret = s5k_check_endpoint(s5k);
	if (ret)
		return ret;

	s5k->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k->regmap))
		return PTR_ERR(s5k->regmap);

	s5k->mclk = devm_clk_get(dev, NULL);
	if (IS_ERR(s5k->mclk))
		return dev_err_probe(dev, PTR_ERR(s5k->mclk), "no mclk\n");
	ret = clk_set_rate(s5k->mclk, S5K_MCLK_HZ);
	if (ret)
		return dev_err_probe(dev, ret, "mclk rate\n");

	s5k->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(s5k->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(s5k->reset_gpio), "reset\n");
	s5k->powerdown_gpio = devm_gpiod_get(dev, "powerdown", GPIOD_OUT_HIGH);
	if (IS_ERR(s5k->powerdown_gpio))
		return dev_err_probe(dev, PTR_ERR(s5k->powerdown_gpio),
				     "powerdown\n");

	for (i = 0; i < ARRAY_SIZE(s5k_supplies); i++)
		s5k->supplies[i].supply = s5k_supplies[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(s5k->supplies),
				      s5k->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");
	s5k->afvdd = devm_regulator_get_optional(dev, "afvdd");
	if (IS_ERR(s5k->afvdd)) {
		if (PTR_ERR(s5k->afvdd) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(s5k->afvdd),
					     "afvdd\n");
		s5k->afvdd = NULL;
	}

	ret = s5k_power_on(dev);
	if (ret)
		return dev_err_probe(dev, ret, "power on\n");

	ret = s5k_read(s5k, S5K_FW_VERSION, &id);
	if (!ret)
		ret = s5k_read(s5k, S5K_FW_REVISION, &rev);
	if (ret || id != S5K_ID) {
		ret = dev_err_probe(dev, ret ?: -ENODEV,
				    "no S5K4ECGX (id %#llx)\n", id);
		goto err_power;
	}

	ret = s5k_init_controls(s5k);
	if (ret)
		goto err_power;

	s5k->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&s5k->sd.entity, 1, &s5k->pad);
	if (ret)
		goto err_ctrls;

	s5k->sd.state_lock = s5k->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&s5k->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&s5k->sd);
	if (ret)
		goto err_pm;

	pm_runtime_put_autosuspend(dev);
	dev_info(dev, "S5K4ECGX id %#llx rev %#llx%s\n", id, rev,
		 s5k->afvdd ? ", AF supply" : "");
	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);
	v4l2_subdev_cleanup(&s5k->sd);
err_entity:
	media_entity_cleanup(&s5k->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&s5k->ctrls);
err_power:
	s5k_power_off(dev);
	return ret;
}

static void s5k_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4ecgx *s5k = to_s5k(sd);

	v4l2_async_unregister_subdev(sd);
	cancel_delayed_work_sync(&s5k->af_work);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s5k->ctrls);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		s5k_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(s5k_pm_ops, s5k_power_off, s5k_power_on,
				 NULL);

static const struct of_device_id s5k_of_match[] = {
	{ .compatible = "samsung,s5k4ecgx" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5k_of_match);

static struct i2c_driver s5k_driver = {
	.driver = {
		.name = "s5k4ecgx",
		.of_match_table = s5k_of_match,
		.pm = pm_ptr(&s5k_pm_ops),
	},
	.probe = s5k_probe,
	.remove = s5k_remove,
};
module_i2c_driver(s5k_driver);

MODULE_DESCRIPTION("Samsung S5K4ECGX camera sensor");
MODULE_LICENSE("GPL");
