// SPDX-License-Identifier: GPL-2.0
/*
 * The SC2723 PMIC as a regmap over the ADI helpers, with its interrupt
 * controller (ANA_INT at slave offset 0x380, GIC SPI 38) as a regmap-irq
 * chip, and the child nodes populated on top. This is what mainline
 * sprd-sc27xx-spi does for the SC2730/SC2731 over spi-sprd-adi, so the
 * mainline sc27xx RTC and PMIC EIC drivers work unchanged on the SC2723:
 * their register blocks match (checked against the vendor 3.10 drivers
 * and on the hardware).
 *
 * Interrupt sources (vendor __irqs-sc8830.h): 0 ADC, 1 GPIO, 2 RTC,
 * 3 watchdog, 4 FGU, 5 EIC, 6 headset button, 7 audio protect,
 * 8 calibration, 9 TPC/thermal, 10 DCDC overtemperature.
 *
 * Bring-up glue: the FGU, ADC and LDO glue still use the ADI helpers
 * directly, which is fine since every access goes through the same lock.
 */

#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/regmap.h>

#include "adi.h"

#define SC2723_INT(x)		(0x380 + (x))
#define INT_MASK_STATUS		SC2723_INT(0x00)
#define INT_EN			SC2723_INT(0x08)
#define SC2723_NR_IRQS		11

struct sc2723_pmic {
	struct regmap_irq irqs[SC2723_NR_IRQS];
	struct regmap_irq_chip irq_chip;
	struct regmap_irq_chip_data *irq_data;
	int irq;
};

static int sc2723_pmic_reg_read(void *ctx, unsigned int reg,
				unsigned int *val)
{
	return sprd_adi_read(reg, val);
}

static int sc2723_pmic_reg_write(void *ctx, unsigned int reg,
				 unsigned int val)
{
	return sprd_adi_write(reg, val);
}

static const struct regmap_config sc2723_pmic_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0xffc,
	.reg_read = sc2723_pmic_reg_read,
	.reg_write = sc2723_pmic_reg_write,
};

static int sc2723_pmic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc2723_pmic *pmic;
	struct regmap *map;
	int i, ret;

	pmic = devm_kzalloc(dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;
	platform_set_drvdata(pdev, pmic);

	pmic->irq = platform_get_irq(pdev, 0);
	if (pmic->irq < 0)
		return pmic->irq;

	map = devm_regmap_init(dev, NULL, pmic, &sc2723_pmic_regmap_config);
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map), "cannot init regmap\n");

	for (i = 0; i < SC2723_NR_IRQS; i++)
		pmic->irqs[i].mask = BIT(i);
	pmic->irq_chip.name = dev_name(dev);
	pmic->irq_chip.status_base = INT_MASK_STATUS;
	pmic->irq_chip.unmask_base = INT_EN;
	pmic->irq_chip.num_regs = 1;
	pmic->irq_chip.irqs = pmic->irqs;
	pmic->irq_chip.num_irqs = SC2723_NR_IRQS;

	/* masks everything first: the bootloader leaves all sources enabled */
	ret = devm_regmap_add_irq_chip(dev, map, pmic->irq, IRQF_ONESHOT, 0,
				       &pmic->irq_chip, &pmic->irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "cannot add the irq chip\n");

	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	return devm_of_platform_populate(dev);
}

/* power key, volume down and the RTC alarm all come in through here */
static int sc2723_pmic_suspend(struct device *dev)
{
	struct sc2723_pmic *pmic = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(pmic->irq);
	return 0;
}

static int sc2723_pmic_resume(struct device *dev)
{
	struct sc2723_pmic *pmic = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(pmic->irq);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sc2723_pmic_pm_ops,
				sc2723_pmic_suspend, sc2723_pmic_resume);

static const struct of_device_id sc2723_pmic_of_match[] = {
	{ .compatible = "sprd,sc2723-adi-pmic" },
	{ }
};

static struct platform_driver sc2723_pmic_driver = {
	.probe = sc2723_pmic_probe,
	.driver = {
		.name = "sc2723-adi-pmic",
		.of_match_table = sc2723_pmic_of_match,
		.pm = pm_sleep_ptr(&sc2723_pmic_pm_ops),
	},
};
builtin_platform_driver(sc2723_pmic_driver);
