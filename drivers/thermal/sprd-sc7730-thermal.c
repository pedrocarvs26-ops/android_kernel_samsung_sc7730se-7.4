// SPDX-License-Identifier: GPL-2.0
/*
 * On-die temperature sensor of the Spreadtrum SC7730/SC8830 ("shark")
 * application processor (vendor: sprd_thm with the SC2713S backend).
 *
 * The sensor block at 0x402f0000 is gated off after the bootloader. Its
 * gate, its RTC clock and its reset sit in AON APB ("aon-apb" region,
 * shared with other drivers, so mapped without request_mem_region). Once
 * started the sensor converts every 2.25 s; the reading is linear from
 * 623 at -40 C to 1030 at 120 C (vendor table, no calibration).
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#define THM_CTRL		0x00
#define  THM_SEN0_EN		BIT(0)
#define THM_INT_CTRL		0x04
#define SENSOR_CTRL		0x20
#define  SENSOR_EN		BIT(0)
#define  SENSOR_SET_RDY		BIT(3)
#define  SENSOR_VENDOR_CFG	0x161	/* enable + detect setup (vendor) */
#define SENSOR_DET_PERI		0x24
#define  SENSOR_DET_2_25S	BIT(13)
#define SENSOR_INT_CTRL		0x28
#define SENSOR_INT_CLR		0x34
#define SENSOR_TEMPER0_READ	0x60
#define  SENSOR_RAW		GENMASK(14, 0)
#define SENSOR_READ_STATUS	0x70
/* sensor 1 shares the detect setting (vendor: "from HW") */
#define SENSOR1_CTRL		0x120
#define  SENSOR1_SEN_DET	0x60

#define AON_APB_EB1		0x04
#define  AON_APB_THM_EB		BIT(1)
#define AON_APB_RST1		0x0c
#define  AON_APB_THM_RST	BIT(1)
#define AON_APB_RTC_EB		0x10
#define  AON_APB_THM_RTC_EB	(BIT(10) | BIT(11) | BIT(13))

#define RAW_AT_MINUS_40		623
#define RAW_AT_120		1030

struct sc7730_thm {
	void __iomem *base;
};

static void sc7730_thm_setbits(void __iomem *reg, u32 bits)
{
	writel(readl(reg) | bits, reg);
}

static void sc7730_thm_clrbits(void __iomem *reg, u32 bits)
{
	writel(readl(reg) & ~bits, reg);
}

static int sc7730_thm_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct sc7730_thm *thm = thermal_zone_device_priv(tz);
	int raw = readl(thm->base + SENSOR_TEMPER0_READ) & SENSOR_RAW;

	/* zero until the first conversion */
	if (!raw)
		return -EAGAIN;

	*temp = -40000 + (raw - RAW_AT_MINUS_40) * 160000 /
		(RAW_AT_120 - RAW_AT_MINUS_40);
	return 0;
}

static const struct thermal_zone_device_ops sc7730_thm_ops = {
	.get_temp = sc7730_thm_get_temp,
};

static int sc7730_thm_aon_init(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *aon;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "aon-apb");
	if (!res)
		return -EINVAL;
	aon = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!aon)
		return -ENOMEM;

	sc7730_thm_setbits(aon + AON_APB_EB1, AON_APB_THM_EB);
	sc7730_thm_setbits(aon + AON_APB_RTC_EB, AON_APB_THM_RTC_EB);
	sc7730_thm_setbits(aon + AON_APB_RST1, AON_APB_THM_RST);
	udelay(10);
	sc7730_thm_clrbits(aon + AON_APB_RST1, AON_APB_THM_RST);
	return 0;
}

static int sc7730_thm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct thermal_zone_device *tz;
	struct sc7730_thm *thm;
	int ret;

	thm = devm_kzalloc(dev, sizeof(*thm), GFP_KERNEL);
	if (!thm)
		return -ENOMEM;

	thm->base = devm_platform_ioremap_resource_byname(pdev, "thm");
	if (IS_ERR(thm->base))
		return PTR_ERR(thm->base);

	ret = sc7730_thm_aon_init(pdev);
	if (ret)
		return dev_err_probe(dev, ret, "no AON APB region\n");

	/* polled: no trip interrupts */
	sc7730_thm_setbits(thm->base + THM_CTRL, THM_SEN0_EN);
	writel(0, thm->base + SENSOR_INT_CTRL);
	writel(~0, thm->base + SENSOR_INT_CLR);
	sc7730_thm_setbits(thm->base + SENSOR_DET_PERI, SENSOR_DET_2_25S);
	sc7730_thm_setbits(thm->base + SENSOR1_CTRL, SENSOR1_SEN_DET);
	sc7730_thm_setbits(thm->base + SENSOR_CTRL, SENSOR_VENDOR_CFG);
	sc7730_thm_setbits(thm->base + SENSOR_CTRL, SENSOR_SET_RDY);

	tz = devm_thermal_of_zone_register(dev, 0, thm, &sc7730_thm_ops);
	if (IS_ERR(tz))
		return dev_err_probe(dev, PTR_ERR(tz),
				     "failed to register thermal zone\n");
	return 0;
}

static const struct of_device_id sc7730_thm_of_match[] = {
	{ .compatible = "sprd,sc7730-thermal" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc7730_thm_of_match);

static struct platform_driver sc7730_thm_driver = {
	.probe = sc7730_thm_probe,
	.driver = {
		.name = "sprd-sc7730-thermal",
		.of_match_table = sc7730_thm_of_match,
	},
};
module_platform_driver(sc7730_thm_driver);

MODULE_DESCRIPTION("Spreadtrum SC7730 on-die temperature sensor");
MODULE_LICENSE("GPL");
