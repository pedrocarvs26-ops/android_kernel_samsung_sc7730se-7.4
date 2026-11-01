// SPDX-License-Identifier: GPL-2.0
/*
 * One LDO of the SC2723 PMIC as a regulator, one device tree node each:
 * a power-down bit ("sprd,pd-reg" = <offset bit>, 1 = off) and a voltage
 * field ("sprd,vsel-reg" = <offset mask>), V = min + n * step with the
 * vendor's defaults of 1.2 V and 10 mV. Registers from the vendor 3.10
 * driver (drivers/regulator/sc2723-regulator_dt.c) and vendor device tree.
 *
 * The power-down and voltage registers are shared between LDOs (and with
 * mach-sprd/usb-bringup.c), so every write is a read-modify-write under
 * the ADI lock. The calibration trim the vendor applies is left alone.
 *
 * Bring-up glue until mainline sc2731-regulator (same idea, other
 * offsets) learns the SC2723 on top of an sc27xx MFD.
 */

#include <linux/bits.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#include "adi.h"

struct sc2723_ldo {
	u32 pd_reg, pd_mask;
	u32 vsel_reg, vsel_mask, vsel_shift;
	struct regulator_desc desc;
};

static int sc2723_ldo_enable(struct regulator_dev *rdev)
{
	struct sc2723_ldo *ldo = rdev_get_drvdata(rdev);

	return sprd_adi_update(ldo->pd_reg, ldo->pd_mask, 0);
}

static int sc2723_ldo_disable(struct regulator_dev *rdev)
{
	struct sc2723_ldo *ldo = rdev_get_drvdata(rdev);

	return sprd_adi_update(ldo->pd_reg, 0, ldo->pd_mask);
}

static int sc2723_ldo_is_enabled(struct regulator_dev *rdev)
{
	struct sc2723_ldo *ldo = rdev_get_drvdata(rdev);
	u32 val;
	int ret;

	ret = sprd_adi_read(ldo->pd_reg, &val);
	return ret ?: !(val & ldo->pd_mask);
}

static int sc2723_ldo_get_voltage_sel(struct regulator_dev *rdev)
{
	struct sc2723_ldo *ldo = rdev_get_drvdata(rdev);
	u32 val;
	int ret;

	ret = sprd_adi_read(ldo->vsel_reg, &val);
	return ret ?: (int)((val & ldo->vsel_mask) >> ldo->vsel_shift);
}

static int sc2723_ldo_set_voltage_sel(struct regulator_dev *rdev,
				      unsigned int sel)
{
	struct sc2723_ldo *ldo = rdev_get_drvdata(rdev);

	return sprd_adi_update(ldo->vsel_reg, ldo->vsel_mask,
			       sel << ldo->vsel_shift);
}

static const struct regulator_ops sc2723_ldo_ops = {
	.enable = sc2723_ldo_enable,
	.disable = sc2723_ldo_disable,
	.is_enabled = sc2723_ldo_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = sc2723_ldo_get_voltage_sel,
	.set_voltage_sel = sc2723_ldo_set_voltage_sel,
};

static int sc2723_ldo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct regulator_config cfg = {};
	struct regulator_dev *rdev;
	struct sc2723_ldo *ldo;
	u32 pd[2], vsel[2], min_uv = 1200000, step_uv = 10000;

	ldo = devm_kzalloc(dev, sizeof(*ldo), GFP_KERNEL);
	if (!ldo)
		return -ENOMEM;

	if (of_property_read_u32_array(np, "sprd,pd-reg", pd, 2) ||
	    of_property_read_u32_array(np, "sprd,vsel-reg", vsel, 2) ||
	    pd[1] > 15 || !vsel[1])
		return dev_err_probe(dev, -EINVAL, "bad sprd,pd-reg/vsel-reg\n");
	of_property_read_u32(np, "sprd,min-microvolt", &min_uv);
	of_property_read_u32(np, "sprd,step-microvolt", &step_uv);

	ldo->pd_reg = pd[0];
	ldo->pd_mask = BIT(pd[1]);
	ldo->vsel_reg = vsel[0];
	ldo->vsel_mask = vsel[1];
	ldo->vsel_shift = __ffs(vsel[1]);

	ldo->desc.name = np->name;
	ldo->desc.type = REGULATOR_VOLTAGE;
	ldo->desc.owner = THIS_MODULE;
	ldo->desc.ops = &sc2723_ldo_ops;
	ldo->desc.min_uV = min_uv;
	ldo->desc.uV_step = step_uv;
	ldo->desc.n_voltages = (vsel[1] >> ldo->vsel_shift) + 1;
	ldo->desc.enable_time = 1000;

	cfg.dev = dev;
	cfg.of_node = np;
	cfg.driver_data = ldo;
	cfg.init_data = of_get_regulator_init_data(dev, np, &ldo->desc);
	if (!cfg.init_data)
		return -ENOMEM;

	rdev = devm_regulator_register(dev, &ldo->desc, &cfg);
	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id sc2723_ldo_of_match[] = {
	{ .compatible = "sprd,sc2723-ldo" },
	{ }
};

static struct platform_driver sc2723_ldo_driver = {
	.probe = sc2723_ldo_probe,
	.driver = {
		.name = "sc2723-ldo",
		.of_match_table = sc2723_ldo_of_match,
	},
};
builtin_platform_driver(sc2723_ldo_driver);
