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
 * former mainline driver did. Only the preview path is supported; the
 * capture sizes of the HAL (up to 2576x1932) need its capture sequence.
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

#define S5K_MCLK_HZ		24000000
#define S5K_LANES		2

struct s5k_mode {
	unsigned int width, height;
	const char *fw;
};

/* 344 Mbit/s per lane, DDR, set up by the init table for all sizes */
static const s64 s5k_link_freqs[] = { 172000000 };

static const struct s5k_mode s5k_modes[] = {
	{ 640, 480, "s5k4ecgx/640x480.bin" },
	{ 800, 480, "s5k4ecgx/800x480.bin" },
	{ 960, 960, "s5k4ecgx/960x960.bin" },
	{ 1024, 768, "s5k4ecgx/1024x768.bin" },
	{ 1280, 720, "s5k4ecgx/1280x720.bin" },
	{ 1280, 768, "s5k4ecgx/1280x768.bin" },
};

static const char * const s5k_supplies[] = { "dovdd", "avdd", "dvdd" };

struct s5k4ecgx {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdown_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(s5k_supplies)];
	struct v4l2_ctrl_handler ctrls;
	const struct s5k_mode *mode;
	const struct s5k_mode *loaded;	/* tables in the ISP, NULL after power off */
	bool powered;			/* by pre_streamon */
};

static inline struct s5k4ecgx *to_s5k(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k4ecgx, sd);
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

/*
 * Write a HAL table. With skip_enable, data words that would land on
 * REG_G_ENABLE_PREV or REG_G_ENABLE_PREV_CHG are left out.
 */
static int s5k_write_table(struct s5k4ecgx *s5k, const char *name,
			   bool skip_enable)
{
	const struct firmware *fw;
	const __le16 *p;
	u32 addr = 0;
	size_t i, n;
	int ret;

	ret = request_firmware(&fw, name, s5k->dev);
	if (ret) {
		dev_err(s5k->dev, "%s missing (extract it from the stock HAL)\n",
			name);
		return ret;
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
			bool skip = skip_enable &&
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
	int ret = 0;

	cci_write(s5k->regmap, S5K_REG_AHB_PAGE, S5K_AHB_PAGE, &ret);
	if (!ret)
		ret = s5k_write(s5k, S5K_G_ENABLE_PREV, on);
	if (!ret)
		ret = s5k_write(s5k, S5K_G_ENABLE_PREV_CHG, 1);
	return ret;
}

static int s5k_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4ecgx *s5k = to_s5k(sd);
	int ret;

	gpiod_set_value_cansleep(s5k->reset_gpio, 1);
	gpiod_set_value_cansleep(s5k->powerdown_gpio, 1);
	/* vendor order: I/O, analog, MCLK, then the external 1.2 V core */
	ret = regulator_enable(s5k->supplies[0].consumer);
	if (ret)
		return ret;
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
	s5k->loaded = NULL;
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
	    fse->index >= ARRAY_SIZE(s5k_modes))
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

	mode = v4l2_find_nearest_size(s5k_modes, ARRAY_SIZE(s5k_modes),
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

/* powered, preview off: the lanes idle in LP-11 */
static int s5k_pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct s5k4ecgx *s5k = to_s5k(sd);
	int ret;

	if (s5k->powered)
		return 0;
	ret = pm_runtime_resume_and_get(s5k->dev);
	if (ret)
		return ret;
	s5k->powered = true;
	return 0;
}

static int s5k_post_streamoff(struct v4l2_subdev *sd)
{
	struct s5k4ecgx *s5k = to_s5k(sd);

	if (s5k->powered) {
		s5k->powered = false;
		pm_runtime_put_autosuspend(s5k->dev);
	}
	return 0;
}

static int s5k_enable_streams(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state, u32 pad,
			      u64 streams_mask)
{
	struct s5k4ecgx *s5k = to_s5k(sd);
	int ret;

	ret = s5k_pre_streamon(sd, 0);
	if (ret)
		return ret;
	/* the init resets the ISP; the size tables differ in more than size */
	if (s5k->loaded != s5k->mode) {
		s5k->loaded = NULL;
		ret = s5k_write_table(s5k, "s5k4ecgx/init.bin", true);
		if (!ret)
			ret = s5k_write_table(s5k, s5k->mode->fw, true);
		if (ret)
			return ret;
		s5k->loaded = s5k->mode;
	}
	return s5k_preview(s5k, true);
}

static int s5k_disable_streams(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state, u32 pad,
			       u64 streams_mask)
{
	return s5k_preview(to_s5k(sd), false);
}

static int s5k_s_ctrl(struct v4l2_ctrl *ctrl)
{
	/* only read-only controls */
	return 0;
}

static const struct v4l2_ctrl_ops s5k_ctrl_ops = {
	.s_ctrl = s5k_s_ctrl,
};

static const struct v4l2_subdev_video_ops s5k_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
	.pre_streamon = s5k_pre_streamon,
	.post_streamoff = s5k_post_streamoff,
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

	v4l2_ctrl_handler_init(hdl, 4);
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
	dev_info(dev, "S5K4ECGX id %#llx rev %#llx\n", id, rev);
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
