// SPDX-License-Identifier: GPL-2.0
/*
 * SiliconFile SR200PC20M, 2 MP YUV camera sensor with its own ISP (front
 * camera of the Samsung Galaxy Tab E 9.6), MIPI CSI-2 with one lane.
 *
 * The register settings exist only in the stock Android camera HAL, so the
 * driver loads them as firmware: sr200pc20m/init.bin plus one file per
 * mode (tools/camtables.py in the gtelwifi bring-up extracts them from the
 * device's own system image). Files are little-endian {u16 reg, u16 val}
 * pairs; registers are 8 bit, paged through 0x03, and 0xff NN is a delay
 * of NN * 10 ms (0xff 0xff is written like any register, as the HAL does).
 *
 * Every table ends by waking the sensor (page 0 register 0x01 = 0x30), so
 * the stream starts at its end. pre_streamon puts the sensor back to sleep
 * right after the tables, which keeps the data lane in LP-11 while the
 * receiver sets up its D-PHY; enable_streams wakes it.
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

#define SR200_REG_PAGE		CCI_REG8(0x03)
#define SR200_REG_SLEEP		CCI_REG8(0x01)
#define  SR200_SLEEP		0x31
#define  SR200_RUN		0x30
#define SR200_REG_ID		CCI_REG8(0x04)
#define  SR200_ID		0xb4
/* image adjustments, page 0x10 */
#define SR200_PAGE_ISP		0x10
#define SR200_REG_ISPCTL4	CCI_REG8(0x13)
#define  SR200_CONTRAST_EN	0x02
#define SR200_REG_YOFS		CCI_REG8(0x40)	/* brightness, sign-magnitude */
#define SR200_REG_CONTRAST	CCI_REG8(0x48)	/* 0x80 = unchanged */
#define SR200_REG_SATB		CCI_REG8(0x61)
#define SR200_REG_SATR		CCI_REG8(0x62)
/* Cb/Cr gain; the vendor tuning has 0x7b, which looks washed out */
#define SR200_SATURATION_DEF	0x7b

#define SR200_MCLK_HZ		24000000
#define SR200_LANES		1

struct sr200_mode {
	unsigned int width, height;
	const char *fw;			/* mode table */
	bool init;			/* sr200pc20m/init.bin first */
	unsigned int link_freq_idx;
};

/* lane rates of the HAL: 192, 288 and 384 Mbit/s, DDR */
static const s64 sr200_link_freqs[] = { 96000000, 144000000, 192000000 };

static const struct sr200_mode sr200_modes[] = {
	{ 800, 600, "sr200pc20m/800x600.bin", true, 0 },
	/* the HAL's 640x480 table is a complete init of its own */
	{ 640, 480, "sr200pc20m/640x480.bin", false, 1 },
	{ 1600, 1200, "sr200pc20m/1600x1200.bin", true, 2 },
};

static const char * const sr200_supplies[] = { "avdd", "dovdd" };

struct sr200 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdown_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(sr200_supplies)];
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	const struct sr200_mode *mode;
	bool prepared;			/* tables written by pre_streamon */
};

static inline struct sr200 *to_sr200(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sr200, sd);
}

static int sr200_write_table(struct sr200 *sr, const char *name)
{
	const struct firmware *fw;
	const __le16 *p;
	size_t i, n;
	int ret;

	ret = request_firmware(&fw, name, sr->dev);
	if (ret) {
		dev_err(sr->dev, "%s missing (extract it from the stock HAL)\n",
			name);
		return ret;
	}
	if (!fw->size || fw->size % 4) {
		dev_err(sr->dev, "%s: bad size %zu\n", name, fw->size);
		release_firmware(fw);
		return -EINVAL;
	}

	p = (const __le16 *)fw->data;
	n = fw->size / 4;
	for (i = 0; i < n && !ret; i++) {
		u16 reg = le16_to_cpu(p[2 * i]), val = le16_to_cpu(p[2 * i + 1]);

		if (reg > 0xff || val > 0xff) {
			ret = -EINVAL;
		} else if (reg == 0xff && val != 0xff) {
			msleep(val * 10);
		} else {
			ret = cci_write(sr->regmap, CCI_REG8(reg), val, NULL);
		}
	}
	release_firmware(fw);
	if (ret)
		dev_err(sr->dev, "%s: entry %zu failed: %d\n", name, i - 1, ret);
	return ret;
}

static int sr200_sleep(struct sr200 *sr, bool sleep)
{
	int ret = 0;

	cci_write(sr->regmap, SR200_REG_PAGE, 0x00, &ret);
	cci_write(sr->regmap, SR200_REG_SLEEP, sleep ? SR200_SLEEP : SR200_RUN,
		  &ret);
	return ret;
}

static int sr200_load_mode(struct sr200 *sr)
{
	int ret = 0;

	if (sr->mode->init)
		ret = sr200_write_table(sr, "sr200pc20m/init.bin");
	if (!ret)
		ret = sr200_write_table(sr, sr->mode->fw);
	return ret;
}

static int sr200_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sr200 *sr = to_sr200(sd);
	int ret;

	gpiod_set_value_cansleep(sr->reset_gpio, 1);
	gpiod_set_value_cansleep(sr->powerdown_gpio, 1);
	ret = regulator_bulk_enable(ARRAY_SIZE(sr->supplies), sr->supplies);
	if (ret)
		return ret;
	usleep_range(3000, 4000);
	gpiod_set_value_cansleep(sr->powerdown_gpio, 0);
	usleep_range(2000, 3000);
	ret = clk_prepare_enable(sr->mclk);
	if (ret) {
		gpiod_set_value_cansleep(sr->powerdown_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(sr->supplies), sr->supplies);
		return ret;
	}
	/* vendor: 30 ms of MCLK before reset is released, then 10 ms */
	msleep(30);
	gpiod_set_value_cansleep(sr->reset_gpio, 0);
	msleep(10);
	return 0;
}

static int sr200_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sr200 *sr = to_sr200(sd);

	sr200_sleep(sr, true);
	gpiod_set_value_cansleep(sr->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(sr->powerdown_gpio, 1);
	clk_disable_unprepare(sr->mclk);
	regulator_bulk_disable(ARRAY_SIZE(sr->supplies), sr->supplies);
	return 0;
}

static void sr200_update_ctrls(struct sr200 *sr)
{
	s64 f = sr200_link_freqs[sr->mode->link_freq_idx];

	__v4l2_ctrl_s_ctrl(sr->link_freq, sr->mode->link_freq_idx);
	/* 16 bits per pixel on one DDR lane */
	__v4l2_ctrl_s_ctrl_int64(sr->pixel_rate, f * 2 * SR200_LANES / 16);
}

static void sr200_fill_fmt(const struct sr200_mode *mode,
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

static int sr200_init_state(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state)
{
	sr200_fill_fmt(&sr200_modes[0], v4l2_subdev_state_get_format(state, 0));
	return 0;
}

static int sr200_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_UYVY8_1X16;
	return 0;
}

static int sr200_enum_frame_size(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_UYVY8_1X16 ||
	    fse->index >= ARRAY_SIZE(sr200_modes))
		return -EINVAL;
	fse->min_width = fse->max_width = sr200_modes[fse->index].width;
	fse->min_height = fse->max_height = sr200_modes[fse->index].height;
	return 0;
}

static int sr200_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *format)
{
	struct sr200 *sr = to_sr200(sd);
	const struct sr200_mode *mode;

	mode = v4l2_find_nearest_size(sr200_modes, ARRAY_SIZE(sr200_modes),
				      width, height, format->format.width,
				      format->format.height);
	sr200_fill_fmt(mode, &format->format);
	*v4l2_subdev_state_get_format(state, 0) = format->format;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sr->mode = mode;
		sr200_update_ctrls(sr);
	}
	return 0;
}

static int sr200_get_selection(struct v4l2_subdev *sd,
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

/*
 * Tables and controls written, sensor asleep: the lane idles in LP-11.
 * Called with the control lock held, which also serializes the paged
 * register writes against sr200_s_ctrl().
 */
static int sr200_prepare(struct sr200 *sr)
{
	int ret;

	if (sr->prepared)
		return 0;
	ret = pm_runtime_resume_and_get(sr->dev);
	if (ret)
		return ret;
	ret = sr200_load_mode(sr);
	if (!ret)
		ret = __v4l2_ctrl_handler_setup(&sr->ctrls);
	if (!ret)
		ret = sr200_sleep(sr, true);
	if (ret) {
		pm_runtime_put(sr->dev);
		return ret;
	}
	sr->prepared = true;
	return 0;
}

static int sr200_pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct sr200 *sr = to_sr200(sd);
	int ret;

	mutex_lock(sr->ctrls.lock);
	ret = sr200_prepare(sr);
	mutex_unlock(sr->ctrls.lock);
	return ret;
}

static int sr200_post_streamoff(struct v4l2_subdev *sd)
{
	struct sr200 *sr = to_sr200(sd);

	if (sr->prepared) {
		sr->prepared = false;
		pm_runtime_put_autosuspend(sr->dev);
	}
	return 0;
}

static int sr200_enable_streams(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state, u32 pad,
				u64 streams_mask)
{
	struct sr200 *sr = to_sr200(sd);
	int ret;

	/* receivers without pre_streamon get the tables now */
	ret = sr200_prepare(sr);
	if (!ret)
		ret = sr200_sleep(sr, false);
	return ret;
}

static int sr200_disable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	return sr200_sleep(to_sr200(sd), true);
}

static int sr200_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sr200 *sr = container_of(ctrl->handler, struct sr200, ctrls);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
		break;
	default:
		/* link frequency and pixel rate follow the mode */
		return 0;
	}

	/* powered off, the tables reset them: written on the next start */
	if (!pm_runtime_get_if_in_use(sr->dev))
		return 0;

	cci_write(sr->regmap, SR200_REG_PAGE, SR200_PAGE_ISP, &ret);
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		cci_write(sr->regmap, SR200_REG_YOFS, ctrl->val < 0 ?
			  0x80 | -ctrl->val : ctrl->val, &ret);
		break;
	case V4L2_CID_CONTRAST:
		cci_write(sr->regmap, SR200_REG_ISPCTL4, SR200_CONTRAST_EN, &ret);
		cci_write(sr->regmap, SR200_REG_CONTRAST, ctrl->val, &ret);
		break;
	case V4L2_CID_SATURATION:
		cci_write(sr->regmap, SR200_REG_SATB, ctrl->val, &ret);
		cci_write(sr->regmap, SR200_REG_SATR, ctrl->val, &ret);
		break;
	}

	pm_runtime_put(sr->dev);
	return ret;
}

static const struct v4l2_ctrl_ops sr200_ctrl_ops = {
	.s_ctrl = sr200_s_ctrl,
};

static const struct v4l2_subdev_video_ops sr200_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
	.pre_streamon = sr200_pre_streamon,
	.post_streamoff = sr200_post_streamoff,
};

static const struct v4l2_subdev_pad_ops sr200_pad_ops = {
	.enum_mbus_code = sr200_enum_mbus_code,
	.enum_frame_size = sr200_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = sr200_set_fmt,
	.get_selection = sr200_get_selection,
	.enable_streams = sr200_enable_streams,
	.disable_streams = sr200_disable_streams,
};

static const struct v4l2_subdev_ops sr200_subdev_ops = {
	.video = &sr200_video_ops,
	.pad = &sr200_pad_ops,
};

static const struct v4l2_subdev_internal_ops sr200_internal_ops = {
	.init_state = sr200_init_state,
};

static int sr200_init_controls(struct sr200 *sr)
{
	struct v4l2_ctrl_handler *hdl = &sr->ctrls;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(hdl, 7);
	v4l2_ctrl_new_std(hdl, &sr200_ctrl_ops, V4L2_CID_BRIGHTNESS,
			  -127, 127, 1, 0);
	v4l2_ctrl_new_std(hdl, &sr200_ctrl_ops, V4L2_CID_CONTRAST,
			  0, 255, 1, 0x80);
	v4l2_ctrl_new_std(hdl, &sr200_ctrl_ops, V4L2_CID_SATURATION,
			  0, 255, 1, SR200_SATURATION_DEF);
	sr->link_freq = v4l2_ctrl_new_int_menu(hdl, &sr200_ctrl_ops,
					       V4L2_CID_LINK_FREQ,
					       ARRAY_SIZE(sr200_link_freqs) - 1,
					       0, sr200_link_freqs);
	sr->pixel_rate = v4l2_ctrl_new_std(hdl, &sr200_ctrl_ops,
					   V4L2_CID_PIXEL_RATE, 1, INT_MAX, 1,
					   sr200_link_freqs[0] / 8);
	if (sr->link_freq)
		sr->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (sr->pixel_rate)
		sr->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ret = v4l2_fwnode_device_parse(sr->dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(hdl, &sr200_ctrl_ops, &props);
	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}
	sr->sd.ctrl_handler = hdl;
	return 0;
}

static int sr200_check_endpoint(struct sr200 *sr)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *fw;
	int ret;

	fw = fwnode_graph_get_next_endpoint(dev_fwnode(sr->dev), NULL);
	if (!fw)
		return dev_err_probe(sr->dev, -EINVAL, "no endpoint\n");
	ret = v4l2_fwnode_endpoint_alloc_parse(fw, &ep);
	fwnode_handle_put(fw);
	if (ret)
		return dev_err_probe(sr->dev, ret, "bad endpoint\n");
	if (ep.bus.mipi_csi2.num_data_lanes != SR200_LANES)
		ret = dev_err_probe(sr->dev, -EINVAL, "one data lane only\n");
	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

static int sr200_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sr200 *sr;
	u64 id;
	int i, ret;

	sr = devm_kzalloc(dev, sizeof(*sr), GFP_KERNEL);
	if (!sr)
		return -ENOMEM;
	sr->dev = dev;
	sr->mode = &sr200_modes[0];
	v4l2_i2c_subdev_init(&sr->sd, client, &sr200_subdev_ops);
	sr->sd.internal_ops = &sr200_internal_ops;

	ret = sr200_check_endpoint(sr);
	if (ret)
		return ret;

	sr->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(sr->regmap))
		return PTR_ERR(sr->regmap);

	sr->mclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sr->mclk))
		return dev_err_probe(dev, PTR_ERR(sr->mclk), "no mclk\n");
	ret = clk_set_rate(sr->mclk, SR200_MCLK_HZ);
	if (ret)
		return dev_err_probe(dev, ret, "mclk rate\n");

	sr->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sr->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sr->reset_gpio), "reset\n");
	sr->powerdown_gpio = devm_gpiod_get(dev, "powerdown", GPIOD_OUT_HIGH);
	if (IS_ERR(sr->powerdown_gpio))
		return dev_err_probe(dev, PTR_ERR(sr->powerdown_gpio),
				     "powerdown\n");

	for (i = 0; i < ARRAY_SIZE(sr200_supplies); i++)
		sr->supplies[i].supply = sr200_supplies[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sr->supplies),
				      sr->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");

	ret = sr200_power_on(dev);
	if (ret)
		return dev_err_probe(dev, ret, "power on\n");

	ret = cci_write(sr->regmap, SR200_REG_PAGE, 0x00, NULL);
	if (!ret)
		ret = cci_read(sr->regmap, SR200_REG_ID, &id, NULL);
	if (ret || id != SR200_ID) {
		ret = dev_err_probe(dev, ret ?: -ENODEV,
				    "no SR200PC20M (id %#llx)\n", id);
		goto err_power;
	}

	ret = sr200_init_controls(sr);
	if (ret)
		goto err_power;

	sr->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sr->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sr->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sr->sd.entity, 1, &sr->pad);
	if (ret)
		goto err_ctrls;

	sr->sd.state_lock = sr->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&sr->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&sr->sd);
	if (ret)
		goto err_pm;

	pm_runtime_put_autosuspend(dev);
	dev_info(dev, "SR200PC20M id %#llx\n", id);
	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);
	v4l2_subdev_cleanup(&sr->sd);
err_entity:
	media_entity_cleanup(&sr->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&sr->ctrls);
err_power:
	sr200_power_off(dev);
	return ret;
}

static void sr200_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sr200 *sr = to_sr200(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sr->ctrls);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		sr200_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(sr200_pm_ops, sr200_power_off,
				 sr200_power_on, NULL);

static const struct of_device_id sr200_of_match[] = {
	{ .compatible = "siliconfile,sr200pc20m" },
	{ }
};
MODULE_DEVICE_TABLE(of, sr200_of_match);

static struct i2c_driver sr200_driver = {
	.driver = {
		.name = "sr200pc20m",
		.of_match_table = sr200_of_match,
		.pm = pm_ptr(&sr200_pm_ops),
	},
	.probe = sr200_probe,
	.remove = sr200_remove,
};
module_i2c_driver(sr200_driver);

MODULE_DESCRIPTION("SiliconFile SR200PC20M camera sensor");
MODULE_LICENSE("GPL");
