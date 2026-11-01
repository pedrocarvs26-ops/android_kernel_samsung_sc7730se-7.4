// SPDX-License-Identifier: GPL-2.0
/*
 * Auxiliary ADC of the SC2723 PMIC as an IIO device, raw counts only.
 *
 * One software conversion at a time, as the vendor 3.10 kernel does it
 * (arch/arm/mach-sc/adc.c): select the input in SW_CH_CFG (small scale,
 * quick mode, resistance path), start a single 12-bit sample with
 * SW_CH_ON and wait for the raw interrupt status. On the gtelwifi the
 * battery thermistor is channel 1. The vendor kernel shares the block
 * with the modem CPU under a hardware spinlock; nothing else uses it here.
 *
 * Bring-up glue on top of the ADI helpers, like the other SC2723 drivers
 * in this directory, until mainline sc27xx_adc knows the SC2723 (other
 * global register offsets, scale in bit 5 instead of bits 10:9).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include "adi.h"

#define SC2723_ADC(x)		(0x300 + (x))
#define ADC_CTL			SC2723_ADC(0x00)
#define ADC_CTL_EN		BIT(0)
#define ADC_CTL_SW_CH_ON	BIT(1)
#define ADC_CTL_12BIT		BIT(2)
#define ADC_CTL_RUN_NUM		GENMASK(7, 4)	/* samples - 1 */
#define ADC_SW_CH_CFG		SC2723_ADC(0x04)
#define ADC_CH_ID		GENMASK(4, 0)
#define ADC_DAT			SC2723_ADC(0x4c)
#define ADC_IRQ_CLR		SC2723_ADC(0x54)
#define ADC_IRQ_RAW		SC2723_ADC(0x5c)

#define GLB_ARM_MODULE_EN	SC2723_GLB(0x000)
#define GLB_ADC_EN		BIT(6)
#define GLB_ARM_CLK_EN		SC2723_GLB(0x004)
#define GLB_CLK_AUXADC_EN	(BIT(9) | BIT(8))
#define GLB_XTL_WAIT_CTRL	SC2723_GLB(0x0e8)
#define GLB_XTL_EN		BIT(8)

#define SC2723_ADC_CHANNELS	32

struct sc2723_adc {
	struct mutex lock;	/* one conversion at a time */
};

static int sc2723_adc_convert(struct sc2723_adc *adc, int channel, int *raw)
{
	u32 ctl, val = 0;
	int i, ret;

	mutex_lock(&adc->lock);
	ret = sprd_adi_write(ADC_SW_CH_CFG, FIELD_PREP(ADC_CH_ID, channel));
	if (!ret)
		ret = sprd_adi_read(ADC_CTL, &ctl);
	if (ret)
		goto out;

	ctl &= ~(ADC_CTL_EN | ADC_CTL_SW_CH_ON | ADC_CTL_12BIT | ADC_CTL_RUN_NUM);
	sprd_adi_write(ADC_CTL, ctl);
	sprd_adi_write(ADC_IRQ_CLR, 1);
	sprd_adi_write(ADC_CTL, ctl | ADC_CTL_EN | ADC_CTL_12BIT |
		       ADC_CTL_SW_CH_ON);

	/* the vendor waits up to 600 us */
	for (i = 0; i < 20; i++) {
		ret = sprd_adi_read(ADC_IRQ_RAW, &val);
		if (ret || val)
			break;
		usleep_range(50, 100);
	}
	if (!ret && !val)
		ret = -ETIMEDOUT;
	if (!ret)
		ret = sprd_adi_read(ADC_DAT, &val);
	if (!ret)
		*raw = val & 0xfff;
	sprd_adi_write(ADC_CTL, ctl);
out:
	mutex_unlock(&adc->lock);
	return ret;
}

static int sc2723_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	int ret;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;
	ret = sc2723_adc_convert(iio_priv(indio_dev), chan->channel, val);
	return ret ?: IIO_VAL_INT;
}

static const struct iio_info sc2723_adc_info = {
	.read_raw = sc2723_adc_read_raw,
};

static int sc2723_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_chan_spec *chans;
	struct iio_dev *indio_dev;
	struct sc2723_adc *adc;
	int i, ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	ret = devm_mutex_init(dev, &adc->lock);
	if (ret)
		return ret;

	chans = devm_kcalloc(dev, SC2723_ADC_CHANNELS, sizeof(*chans),
			     GFP_KERNEL);
	if (!chans)
		return -ENOMEM;
	for (i = 0; i < SC2723_ADC_CHANNELS; i++) {
		chans[i].type = IIO_VOLTAGE;
		chans[i].indexed = 1;
		chans[i].channel = i;
		chans[i].info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	}

	/* module, its clocks and the crystal it samples with */
	ret = sprd_adi_update(GLB_ARM_MODULE_EN, 0, GLB_ADC_EN);
	if (!ret)
		ret = sprd_adi_update(GLB_ARM_CLK_EN, 0, GLB_CLK_AUXADC_EN);
	if (!ret)
		ret = sprd_adi_update(GLB_XTL_WAIT_CTRL, 0, GLB_XTL_EN);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable the ADC\n");

	indio_dev->name = "sc2723-adc";
	indio_dev->info = &sc2723_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chans;
	indio_dev->num_channels = SC2723_ADC_CHANNELS;
	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id sc2723_adc_of_match[] = {
	{ .compatible = "sprd,sc2723-adc-polled" },
	{ }
};

static struct platform_driver sc2723_adc_driver = {
	.probe = sc2723_adc_probe,
	.driver = {
		.name = "sc2723-adc-polled",
		.of_match_table = sc2723_adc_of_match,
	},
};
builtin_platform_driver(sc2723_adc_driver);
