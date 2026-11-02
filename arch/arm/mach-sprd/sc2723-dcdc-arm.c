// SPDX-License-Identifier: GPL-2.0
/*
 * DCDC_ARM of the SC2723 PMIC, the Cortex-A7 cluster supply ("vddarm"),
 * as a regulator for CPU frequency scaling.
 *
 * DCDC_ARM_ADI (0xA04) holds the voltage. With MIXED_CTRL0 (0x918) bit 14
 * clear, which the SPL and sboot assume, bits 7:5 pick a base voltage and
 * bits 4:0 add 3.125 mV steps on top (vendor sc2723-regulator_dt.c,
 * "non-linear" mode). Selectors here are linear from 650 mV in 3.125 mV
 * steps and get mapped onto base + trim. The linear mode (bit 14 set)
 * would put very different voltages on the same register values, so the
 * driver refuses to probe then.
 *
 * Like the vendor driver it moves in 25 mV steps against overshoot,
 * waiting 5 us per step when rising (10 us per 50 mV) and not at all when
 * falling. Doing that here rather than with the core's max_uV_step keeps
 * the core from asking for a stepped minimum above the target maximum
 * when it lowers the voltage, which failed with -EWOULDBLOCK.
 *
 * The rail is never switched: DCDC_ARM_PD (0x810 bit 10) and the OTP/SW
 * select bit (0x8C0 bit 2) are left alone. The vendor kernel also writes
 * 0xA04 with that select bit clear, and the rail follows (measured).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#include "adi.h"

#define DCDC_ARM_ADI		0xa04
#define  DCDC_ARM_CTL		GENMASK(7, 5)
#define  DCDC_ARM_CAL		GENMASK(4, 0)
#define  DCDC_ARM_RSV		GENMASK(9, 8)
#define MIXED_CTRL0		0x918
#define  DCDC_V_CTRL_MODE	BIT(14)

#define DCDC_MIN_UV		650000
#define DCDC_STEP_UV		3125
#define DCDC_N_VOLTAGES		240	/* up to 1396.875 mV */
#define DCDC_STEP_MAX_UV	25000
#define DCDC_RISE_STEP_US	5

/* CTL index -> base voltage in mV (vendor dcdc_vol_select[]) */
static const u16 sc2723_dcdc_base[8] = {
	1100, 700, 800, 900, 1000, 650, 1200, 1300,
};

static int sc2723_dcdc_read_uv(u32 *uv)
{
	u32 val;
	int ret;

	ret = sprd_adi_read(DCDC_ARM_ADI, &val);
	if (ret)
		return ret;

	*uv = sc2723_dcdc_base[FIELD_GET(DCDC_ARM_CTL, val)] * 1000 +
	      FIELD_GET(DCDC_ARM_CAL, val) * DCDC_STEP_UV;
	return 0;
}

static int sc2723_dcdc_get_voltage_sel(struct regulator_dev *rdev)
{
	u32 uv;
	int ret;

	ret = sc2723_dcdc_read_uv(&uv);
	return ret ?: (int)((uv - DCDC_MIN_UV) / DCDC_STEP_UV);
}

/* the highest base at or below the target, within the 31 trim steps */
static int sc2723_dcdc_write_uv(u32 uv)
{
	u32 best = 0, trim;
	int i, idx = -1;

	for (i = 0; i < ARRAY_SIZE(sc2723_dcdc_base); i++) {
		u32 base = sc2723_dcdc_base[i] * 1000;

		if (base <= uv && base >= best) {
			best = base;
			idx = i;
		}
	}
	trim = (uv - best) / DCDC_STEP_UV;
	if (idx < 0 || trim > FIELD_MAX(DCDC_ARM_CAL))
		return -EINVAL;

	return sprd_adi_update(DCDC_ARM_ADI, DCDC_ARM_CTL | DCDC_ARM_CAL,
			       FIELD_PREP(DCDC_ARM_CTL, idx) |
			       FIELD_PREP(DCDC_ARM_CAL, trim));
}

static int sc2723_dcdc_set_voltage_sel(struct regulator_dev *rdev,
				       unsigned int sel)
{
	u32 target = DCDC_MIN_UV + sel * DCDC_STEP_UV, uv;
	int ret;

	ret = sc2723_dcdc_read_uv(&uv);
	while (!ret && uv != target) {
		if (target > uv) {
			uv = min(uv + DCDC_STEP_MAX_UV, target);
			ret = sc2723_dcdc_write_uv(uv);
			udelay(DCDC_RISE_STEP_US);
		} else {
			uv = max(uv - DCDC_STEP_MAX_UV, target);
			ret = sc2723_dcdc_write_uv(uv);
		}
	}
	return ret;
}

static const struct regulator_ops sc2723_dcdc_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = sc2723_dcdc_get_voltage_sel,
	.set_voltage_sel = sc2723_dcdc_set_voltage_sel,
};

static const struct regulator_desc sc2723_dcdc_desc = {
	.name = "vddarm",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &sc2723_dcdc_ops,
	.min_uV = DCDC_MIN_UV,
	.uV_step = DCDC_STEP_UV,
	.n_voltages = DCDC_N_VOLTAGES,
};

static int sc2723_dcdc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config cfg = {};
	struct regulator_dev *rdev;
	u32 mode, val;
	int ret;

	ret = sprd_adi_read(MIXED_CTRL0, &mode);
	if (!ret)
		ret = sprd_adi_read(DCDC_ARM_ADI, &val);
	if (ret)
		return ret;
	if (mode & DCDC_V_CTRL_MODE)
		return dev_err_probe(dev, -ENODEV,
				     "linear voltage mode, not supported\n");
	if (val & DCDC_ARM_RSV)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected DCDC_ARM_ADI %#x\n", val);

	cfg.dev = dev;
	cfg.of_node = dev->of_node;
	cfg.init_data = of_get_regulator_init_data(dev, dev->of_node,
						   &sc2723_dcdc_desc);
	if (!cfg.init_data)
		return -ENOMEM;

	rdev = devm_regulator_register(dev, &sc2723_dcdc_desc, &cfg);
	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id sc2723_dcdc_of_match[] = {
	{ .compatible = "sprd,sc2723-dcdc-arm" },
	{ }
};

static struct platform_driver sc2723_dcdc_driver = {
	.probe = sc2723_dcdc_probe,
	.driver = {
		.name = "sc2723-dcdc-arm",
		.of_match_table = sc2723_dcdc_of_match,
	},
};
builtin_platform_driver(sc2723_dcdc_driver);
