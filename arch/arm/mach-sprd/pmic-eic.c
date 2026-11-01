// SPDX-License-Identifier: GPL-2.0
/*
 * The 16 inputs of the SC2723 PMIC's EIC block as a GPIO chip without
 * interrupts, for gpio-keys-polled: power key (channel 2, PBINT) and
 * volume down (channel 10, the EXT_RSTN pin) on the gtelwifi. Inputs are
 * active high; the hardware debounce stays as the bootloader set it.
 *
 * Bring-up glue on top of the ADI helpers. The proper way is mainline
 * gpio-pmic-eic-sprd on the sc27xx MFD, with interrupts instead of
 * polling, but that first needs SC7730 support in spi-sprd-adi (other
 * register offsets) and SC2723 data in sprd-sc27xx-spi.
 */

#include <linux/bits.h>
#include <linux/gpio/driver.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#include "adi.h"

#define SC2723_EIC(x)		(0x100 + (x))
#define EIC_DATA		SC2723_EIC(0x00)
#define EIC_DMSK		SC2723_EIC(0x04)	/* DATA shows only these */
#define EIC_NR			16

#define GLB_ARM_MODULE_EN	SC2723_GLB(0x00)
#define GLB_RTC_CLK_EN		SC2723_GLB(0x08)
#define GLB_EIC_EN		BIT(3)		/* in both registers */

static int sc2723_eic_get(struct gpio_chip *gc, unsigned int offset)
{
	u32 v;
	int ret;

	ret = sprd_adi_read(EIC_DATA, &v);
	if (ret)
		return ret;
	return !!(v & BIT(offset));
}

static int sc2723_eic_request(struct gpio_chip *gc, unsigned int offset)
{
	return sprd_adi_update(EIC_DMSK, 0, BIT(offset));
}

static int sc2723_eic_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_IN;
}

static int sc2723_eic_direction_input(struct gpio_chip *gc,
				      unsigned int offset)
{
	return 0;
}

static int sc2723_eic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_chip *gc;
	int ret;

	ret = sprd_adi_update(GLB_ARM_MODULE_EN, 0, GLB_EIC_EN);
	if (!ret)
		ret = sprd_adi_update(GLB_RTC_CLK_EN, 0, GLB_EIC_EN);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable the EIC\n");

	gc = devm_kzalloc(dev, sizeof(*gc), GFP_KERNEL);
	if (!gc)
		return -ENOMEM;
	gc->label = "sc2723-eic";
	gc->parent = dev;
	gc->owner = THIS_MODULE;
	gc->base = -1;
	gc->ngpio = EIC_NR;
	gc->request = sc2723_eic_request;
	gc->get = sc2723_eic_get;
	gc->get_direction = sc2723_eic_get_direction;
	gc->direction_input = sc2723_eic_direction_input;
	gc->can_sleep = false;	/* ADI access spins, briefly */

	return devm_gpiochip_add_data(dev, gc, NULL);
}

static const struct of_device_id sc2723_eic_of_match[] = {
	{ .compatible = "sprd,sc2723-eic-polled" },
	{ }
};

static struct platform_driver sc2723_eic_driver = {
	.probe = sc2723_eic_probe,
	.driver = {
		.name = "sc2723-eic-polled",
		.of_match_table = sc2723_eic_of_match,
	},
};
builtin_platform_driver(sc2723_eic_driver);
