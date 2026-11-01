// SPDX-License-Identifier: GPL-2.0
/*
 * Fuel gauge (FGU) of the SC2723 PMIC as a battery power supply, polled.
 *
 * The bootloader switches the FGU on at every boot; this driver only reads
 * it: battery voltage (VOLT_VAL), current through the 20 mOhm sense resistor
 * (CURT_VAL, 8192 = 0 A, positive = charging) and the coulomb counter, which
 * integrates CURT - 8192 at 2 Hz. The ADC scale comes from two calibration
 * points (4.2 V and 3.6 V) in the PMIC's A-die eFuse. State of charge is
 * coulomb counting from an OCV estimate (V - I * R_int) at probe, with the
 * vendor's re-anchoring at the top and bottom of the OCV table and when the
 * charger reports the battery full. Like the vendor, the capacity is kept
 * in the FGU's always-on USER_AREA, so a reboot continues from it instead
 * of a fresh estimate under load.
 *
 * Register layout, eFuse decoding and formulas from the vendor 3.10 driver
 * (drivers/battery/sprd_2713_fgu.c, drivers/misc/sprd_otp/). Mainline
 * sc27xx_fuel_gauge has the same FGU block but needs a regmap parent, IIO
 * channels and an nvmem cell that do not exist for the SC2723 yet; this is
 * bring-up glue on top of the ADI helpers until they do.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/workqueue.h>

#include "adi.h"

#define GLB_ARM_MODULE_EN	SC2723_GLB(0x000)
#define GLB_FGU_EN		BIT(8)
#define GLB_EFS_EN		BIT(7)
#define GLB_RTC_CLK_EN		SC2723_GLB(0x008)
#define GLB_RTC_FGU_EN		(BIT(7) | BIT(6))

#define EFUSE(x)		(0x200 + (x))
#define EFUSE_DATA_RD		EFUSE(0x04)
#define EFUSE_BLOCK_INDEX	EFUSE(0x0c)
#define EFUSE_MODE_CTRL		EFUSE(0x10)
#define EFUSE_RD_START		BIT(1)	/* bit 0 starts programming: never */
#define EFUSE_STATUS		EFUSE(0x14)
#define EFUSE_READ_BUSY		BIT(1)

#define FGU(x)			(0x500 + (x))
#define FGU_VOLT_VAL		FGU(0x20)
#define FGU_CURT_VAL		FGU(0x2c)
#define FGU_CLBCNT_VALH		FGU(0x68)
#define FGU_CLBCNT_VALL		FGU(0x6c)
#define FGU_USER_AREA_SET	FGU(0xa0)
#define FGU_USER_AREA_CLEAR	FGU(0xa4)
#define FGU_USER_AREA		FGU(0xa8)
#define USER_AREA_SOC		GENMASK(11, 0)
#define USER_AREA_TYPE		GENMASK(15, 12)
#define USER_AREA_TYPE_NORMAL	0x5	/* 0xf after a battery disconnect */

#define FGU_CURT_ZERO		8192
#define FGU_POLL_MS		30000
/* a saved capacity this far from the OCV estimate is stale */
#define FGU_SAVED_MAX_DIFF	25

/* without eFuse data (vendor defaults): ADC counts at 4.2 V and 3.6 V */
#define FGU_DEF_A42		2752
#define FGU_DEF_A36		2374

struct sc2723_fgu {
	struct device *dev;
	struct power_supply *psy;
	struct power_supply_battery_info *info;
	struct delayed_work work;
	int vol_1000mv_adc, vol_offset, cur_1000ma_adc;
	int rint_mohm;
	int soc0;		/* capacity at the last anchor, % */
	s32 cc0;		/* coulomb counter at the last anchor */
	int soc;		/* last reported capacity, % */
	int status;
	bool ready;		/* battery data loaded, polling may run */
};

static int fgu_read(u32 reg)
{
	u32 val;

	return sprd_adi_read(reg, &val) ?: (int)val;
}

static int sc2723_efuse_read(int block)
{
	u32 val;
	int i, ret;

	ret = sprd_adi_update(GLB_ARM_MODULE_EN, 0, GLB_EFS_EN);
	if (ret)
		return ret;
	sprd_adi_write(EFUSE_BLOCK_INDEX, block & 0x1f);
	sprd_adi_write(EFUSE_MODE_CTRL, EFUSE_RD_START);
	for (i = 0; i < 1000; i++) {
		ret = sprd_adi_read(EFUSE_STATUS, &val);
		if (ret || !(val & EFUSE_READ_BUSY))
			break;
		udelay(10);
	}
	if (!ret && i == 1000)
		ret = -ETIMEDOUT;
	if (!ret)
		ret = sprd_adi_read(EFUSE_DATA_RD, &val);
	sprd_adi_update(GLB_ARM_MODULE_EN, GLB_EFS_EN, 0);

	/* the blocks read back inverted */
	return ret ?: (int)(~val & 0xff);
}

/* Two calibration points from the eFuse, as the vendor's otp-helper.c */
static void sc2723_fgu_calibrate(struct sc2723_fgu *fgu, u32 r_real_uohm)
{
	int a42 = FGU_DEF_A42, a36 = FGU_DEF_A36, blk[4], i;
	static const int blocks[] = { 0, 12, 13, 14 };
	const char *src = "defaults";
	u32 bits;

	for (i = 0; i < ARRAY_SIZE(blocks); i++)
		blk[i] = sc2723_efuse_read(blocks[i]);

	if (blk[0] >= 0 && blk[1] >= 0 && blk[2] >= 0 && blk[3] >= 0 &&
	    !(blk[0] & BIT(7))) {
		/* blocks 12..14 as one little-endian bit string */
		bits = blk[1] | blk[2] << 8 | blk[3] << 16;
		a42 = (bits & 0x1ff) + 6963 - 4096 - 256;
		a36 = a42 - 410 - ((bits >> 18) & 0x3f) + 16;
		src = "eFuse";
	}

	fgu->vol_1000mv_adc = ((a42 - a36) * 10 + 3) / 6;
	fgu->vol_offset = -(a42 * 10 - fgu->vol_1000mv_adc * 42) / 10;
	/* nominal sense resistor 20 mOhm, the board's real one in DT */
	fgu->cur_1000ma_adc = DIV_ROUND_CLOSEST(fgu->vol_1000mv_adc * 4 *
						r_real_uohm, 20000);

	dev_info(fgu->dev,
		 "%s calibration: 4.2 V = %d, 3.6 V = %d (eFuse %02x %02x %02x %02x), %d/V, offset %d, %d/A\n",
		 src, a42, a36, blk[0], blk[1], blk[2], blk[3],
		 fgu->vol_1000mv_adc, fgu->vol_offset, fgu->cur_1000ma_adc);
}

static int fgu_uv(struct sc2723_fgu *fgu, int raw)
{
	return div_s64((s64)(raw + fgu->vol_offset) * 1000000,
		       fgu->vol_1000mv_adc);
}

static int fgu_ua(struct sc2723_fgu *fgu, int raw)
{
	return div_s64((s64)(raw - FGU_CURT_ZERO) * 1000000,
		       fgu->cur_1000ma_adc);
}

static int sc2723_fgu_clbcnt(s32 *cc)
{
	int h1, l1, h2, l2, i;

	/* read until two snapshots agree, the halves are not latched */
	for (i = 0; i < 5; i++) {
		l1 = fgu_read(FGU_CLBCNT_VALL);
		h1 = fgu_read(FGU_CLBCNT_VALH);
		l2 = fgu_read(FGU_CLBCNT_VALL);
		h2 = fgu_read(FGU_CLBCNT_VALH);
		if (l1 < 0 || h1 < 0 || l2 < 0 || h2 < 0)
			return -EIO;
		if (l1 == l2 && h1 == h2) {
			*cc = (s32)((u32)h1 << 16 | l1);
			return 0;
		}
	}
	return -EAGAIN;
}

/* counts to µAh: the counter adds CURT - 8192 twice per second */
static s64 fgu_cc_to_uah(struct sc2723_fgu *fgu, s32 delta)
{
	return div_s64((s64)delta * 10000, 72 * fgu->cur_1000ma_adc);
}

static int sc2723_fgu_ocv_uv(struct sc2723_fgu *fgu, int uv, int ua)
{
	return uv - div_s64((s64)ua * fgu->rint_mohm, 1000);
}

static void sc2723_fgu_anchor(struct sc2723_fgu *fgu, int soc)
{
	s32 cc;

	if (sc2723_fgu_clbcnt(&cc))
		return;
	fgu->soc0 = soc;
	fgu->cc0 = cc;
}

static void sc2723_fgu_save(int soc)
{
	u32 val = FIELD_PREP(USER_AREA_TYPE, USER_AREA_TYPE_NORMAL) |
		  FIELD_PREP(USER_AREA_SOC, soc);

	/* set/clear pair like the vendor; the status register reads back */
	sprd_adi_write(FGU_USER_AREA_CLEAR, ~val & (USER_AREA_TYPE | USER_AREA_SOC));
	sprd_adi_write(FGU_USER_AREA_SET, val);
}

static bool sc2723_fgu_charger_full(struct sc2723_fgu *fgu)
{
	union power_supply_propval val;

	return !power_supply_get_property_from_supplier(fgu->psy,
				POWER_SUPPLY_PROP_STATUS, &val) &&
	       val.intval == POWER_SUPPLY_STATUS_FULL;
}

static void sc2723_fgu_update(struct sc2723_fgu *fgu)
{
	int volt = fgu_read(FGU_VOLT_VAL), curt = fgu_read(FGU_CURT_VAL);
	int uv, ua, ocv, soc, status, full_uah;
	bool supplied, full;
	s32 cc, delta;

	if (volt < 0 || curt < 0 || sc2723_fgu_clbcnt(&cc))
		return;
	uv = fgu_uv(fgu, volt & 0x1fff);
	ua = fgu_ua(fgu, curt & 0x3fff);
	ocv = sc2723_fgu_ocv_uv(fgu, uv, ua);
	full_uah = fgu->info->charge_full_design_uah;

	/* the counter may be 30 bit wide: compare modulo 2^30 */
	delta = sign_extend32(cc - fgu->cc0, 29);
	soc = fgu->soc0 + div_s64(fgu_cc_to_uah(fgu, delta) * 100, full_uah);

	/*
	 * Re-anchor like the vendor at the ends of the OCV table and when
	 * the charger has terminated. After termination the tablet runs
	 * from the battery until the charger restarts, keep 100 % meanwhile.
	 */
	supplied = power_supply_am_i_supplied(fgu->psy) > 0;
	full = supplied && sc2723_fgu_charger_full(fgu);
	if (full || ocv >= fgu->info->constant_charge_voltage_max_uv)
		soc = 100;
	else if (uv < fgu->info->voltage_min_design_uv)
		soc = 0;
	soc = clamp(soc, 0, 100);
	if (soc == 100 || soc == 0)
		sc2723_fgu_anchor(fgu, soc);

	/*
	 * By the current: in its 500 mA USB mode the charger can supply less
	 * than the tablet draws, then the battery discharges while plugged in.
	 */
	if (full)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (ua > 20000)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else if (!supplied || ua < -20000)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (soc >= 99)
		status = POWER_SUPPLY_STATUS_FULL;
	else
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	if (soc != fgu->soc)
		sc2723_fgu_save(soc);
	if (soc != fgu->soc || status != fgu->status) {
		fgu->soc = soc;
		fgu->status = status;
		power_supply_changed(fgu->psy);
	}
}

static void sc2723_fgu_work(struct work_struct *work)
{
	struct sc2723_fgu *fgu = container_of(work, struct sc2723_fgu,
					      work.work);

	sc2723_fgu_update(fgu);
	queue_delayed_work(system_dfl_wq, &fgu->work,
			   msecs_to_jiffies(FGU_POLL_MS));
}

static int sc2723_fgu_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sc2723_fgu *fgu = power_supply_get_drvdata(psy);
	int raw, curt;
	s32 cc;

	/* registering the supply already queries it */
	if (!READ_ONCE(fgu->ready))
		return -ENODATA;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fgu->status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		raw = fgu_read(FGU_VOLT_VAL);
		if (raw < 0)
			return raw;
		val->intval = fgu_uv(fgu, raw & 0x1fff);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		raw = fgu_read(FGU_CURT_VAL);
		if (raw < 0)
			return raw;
		val->intval = fgu_ua(fgu, raw & 0x3fff);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		raw = fgu_read(FGU_VOLT_VAL);
		curt = fgu_read(FGU_CURT_VAL);
		if (raw < 0 || curt < 0)
			return -EIO;
		val->intval = sc2723_fgu_ocv_uv(fgu, fgu_uv(fgu, raw & 0x1fff),
						fgu_ua(fgu, curt & 0x3fff));
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = fgu->soc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (sc2723_fgu_clbcnt(&cc))
			return -EIO;
		val->intval = fgu->info->charge_full_design_uah / 100 * fgu->soc0 +
			      fgu_cc_to_uah(fgu, sign_extend32(cc - fgu->cc0, 29));
		val->intval = clamp(val->intval, 0,
				    fgu->info->charge_full_design_uah);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = fgu->info->charge_full_design_uah;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = fgu->info->voltage_min_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = fgu->info->constant_charge_voltage_max_uv;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void sc2723_fgu_external_power_changed(struct power_supply *psy)
{
	struct sc2723_fgu *fgu = power_supply_get_drvdata(psy);

	if (READ_ONCE(fgu->ready))
		mod_delayed_work(system_dfl_wq, &fgu->work,
				 msecs_to_jiffies(1000));
}

static const enum power_supply_property sc2723_fgu_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
};

static const struct power_supply_desc sc2723_fgu_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sc2723_fgu_props,
	.num_properties = ARRAY_SIZE(sc2723_fgu_props),
	.get_property = sc2723_fgu_get_property,
	.external_power_changed = sc2723_fgu_external_power_changed,
};

static void sc2723_fgu_stop(void *data)
{
	struct sc2723_fgu *fgu = data;

	WRITE_ONCE(fgu->ready, false);
	cancel_delayed_work_sync(&fgu->work);
}

static int sc2723_fgu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = {};
	struct sc2723_fgu *fgu;
	int volt, curt, uv, ua, ocv, soc, saved, ret;
	u32 r_real = 20000;

	fgu = devm_kzalloc(dev, sizeof(*fgu), GFP_KERNEL);
	if (!fgu)
		return -ENOMEM;
	fgu->dev = dev;
	INIT_DELAYED_WORK(&fgu->work, sc2723_fgu_work);

	/* no-ops after the bootloader, which enables the FGU every boot */
	ret = sprd_adi_update(GLB_ARM_MODULE_EN, 0, GLB_FGU_EN);
	if (!ret)
		ret = sprd_adi_update(GLB_RTC_CLK_EN, 0, GLB_RTC_FGU_EN);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable the FGU\n");

	device_property_read_u32(dev, "sprd,calib-resistance-micro-ohms",
				 &r_real);
	sc2723_fgu_calibrate(fgu, r_real);

	cfg.drv_data = fgu;
	cfg.fwnode = dev_fwnode(dev);
	fgu->psy = devm_power_supply_register(dev, &sc2723_fgu_desc, &cfg);
	if (IS_ERR(fgu->psy))
		return dev_err_probe(dev, PTR_ERR(fgu->psy),
				     "cannot register the battery\n");

	ret = power_supply_get_battery_info(fgu->psy, &fgu->info);
	if (ret)
		return dev_err_probe(dev, ret, "no monitored-battery\n");
	if (fgu->info->charge_full_design_uah <= 0 ||
	    fgu->info->constant_charge_voltage_max_uv <= 0 ||
	    fgu->info->voltage_min_design_uv <= 0 ||
	    !fgu->info->ocv_table_size[0])
		return dev_err_probe(dev, -EINVAL, "incomplete battery data\n");
	fgu->rint_mohm = fgu->info->factory_internal_resistance_uohm > 0 ?
			 fgu->info->factory_internal_resistance_uohm / 1000 : 0;

	/*
	 * Start from the saved capacity, or from the OCV (V - I * R_int)
	 * when there is none or it is stale, e.g. after charging while off.
	 */
	volt = fgu_read(FGU_VOLT_VAL);
	curt = fgu_read(FGU_CURT_VAL);
	saved = fgu_read(FGU_USER_AREA);
	if (volt < 0 || curt < 0 || saved < 0)
		return dev_err_probe(dev, -EIO, "cannot read the FGU\n");
	uv = fgu_uv(fgu, volt & 0x1fff);
	ua = fgu_ua(fgu, curt & 0x3fff);
	ocv = sc2723_fgu_ocv_uv(fgu, uv, ua);
	soc = clamp(power_supply_batinfo_ocv2cap(fgu->info, ocv, 20), 0, 100);
	fgu->soc = soc;
	if (FIELD_GET(USER_AREA_TYPE, saved) == USER_AREA_TYPE_NORMAL &&
	    FIELD_GET(USER_AREA_SOC, saved) <= 100 &&
	    abs((int)FIELD_GET(USER_AREA_SOC, saved) - soc) <= FGU_SAVED_MAX_DIFF)
		fgu->soc = FIELD_GET(USER_AREA_SOC, saved);
	fgu->status = POWER_SUPPLY_STATUS_UNKNOWN;
	sc2723_fgu_anchor(fgu, fgu->soc);
	sc2723_fgu_save(fgu->soc);
	dev_info(dev,
		 "VOLT %d = %d mV, CURT %d = %d mA, OCV %d mV = %d %%, CLBCNT %08x, USER_AREA %04x, start %d %%\n",
		 volt, uv / 1000, curt, ua / 1000, ocv / 1000, soc,
		 fgu->cc0, saved, fgu->soc);

	/* runs before the supply is unregistered (devm order) */
	ret = devm_add_action_or_reset(dev, sc2723_fgu_stop, fgu);
	if (ret)
		return ret;
	WRITE_ONCE(fgu->ready, true);
	queue_delayed_work(system_dfl_wq, &fgu->work, 0);
	return 0;
}

static const struct of_device_id sc2723_fgu_of_match[] = {
	{ .compatible = "sprd,sc2723-fgu-polled" },
	{ }
};

static struct platform_driver sc2723_fgu_driver = {
	.probe = sc2723_fgu_probe,
	.driver = {
		.name = "sc2723-fgu-polled",
		.of_match_table = sc2723_fgu_of_match,
	},
};
builtin_platform_driver(sc2723_fgu_driver);
