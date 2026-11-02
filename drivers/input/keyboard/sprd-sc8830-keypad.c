// SPDX-License-Identifier: GPL-2.0
/*
 * Keypad matrix controller of the Spreadtrum SC8830/SC7730 ("shark") SoCs.
 *
 * The controller scans up to 8 rows (KEYOUT) and 8 columns (KEYIN) and
 * tracks up to four keys at once: each byte of KEY_STATUS holds the row
 * and column of one key, and that slot's press and release bits in the
 * interrupt status say what happened to it. Register layout and init
 * values from the vendor 3.10 driver (drivers/input/keyboard/sc_keypad.c).
 *
 * Only rows and columns that carry a key in linux,keymap are scanned:
 * the other keypad pads may be muxed to GPIO and would float. Until there
 * is a clock driver for this SoC, the AON APB clock gates and soft reset
 * (optional "aon-apb" region) are handled here.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/log2.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/property.h>
#include <linux/slab.h>

#define KPD_CTRL		0x00
#define KPD_CTRL_EN		BIT(0)
#define KPD_CTRL_SLEEP_EN	BIT(1)
#define KPD_CTRL_COLS(m)	((m) << 8)
#define KPD_CTRL_ROWS(m)	((m) << 16)
#define KPD_INT_EN		0x04
#define KPD_INT_MASK_STS	0x0c
#define KPD_INT_CLR		0x10
#define KPD_INT_ALL		0xfff
#define KPD_INT_DOWNUP		0x0ff	/* without the long-key bits 11:8 */
#define KPD_INT_PRESS(n)	BIT(n)
#define KPD_INT_RELEASE(n)	BIT((n) + 4)
#define KPD_POLARITY		0x18
#define KPD_DEBOUNCE_CNT	0x1c
#define KPD_LONG_KEY_CNT	0x20
#define KPD_SLEEP_CNT		0x24
#define KPD_CLK_DIV_CNT		0x28
#define KPD_KEY_STATUS		0x2c
#define KPD_SLOT_COL(s)		((s) & 0x7)
#define KPD_SLOT_ROW(s)		(((s) >> 4) & 0x7)

#define KPD_SLOTS		4
#define KPD_MAX_LINES		8

#define AON_APB_EB0		0x00
#define AON_APB_RST0		0x08
#define AON_APB_RTC_EB		0x10
#define AON_APB_KPD_EB		BIT(8)
#define AON_APB_KPD_RST		BIT(8)
#define AON_APB_KPD_RTC_EB	BIT(1)

struct sc8830_kpd {
	void __iomem *base;
	struct input_dev *input;
	unsigned int rows, cols, row_shift;
};

static void sc8830_kpd_report(struct sc8830_kpd *kpd, u8 slot, int down)
{
	unsigned short *keycodes = kpd->input->keycode;
	unsigned int row = KPD_SLOT_ROW(slot), col = KPD_SLOT_COL(slot);
	unsigned int code;

	if (row >= kpd->rows || col >= kpd->cols)
		return;
	code = MATRIX_SCAN_CODE(row, col, kpd->row_shift);
	input_event(kpd->input, EV_MSC, MSC_SCAN, code);
	input_report_key(kpd->input, keycodes[code], down);
	input_sync(kpd->input);
}

static irqreturn_t sc8830_kpd_irq(int irq, void *dev_id)
{
	struct sc8830_kpd *kpd = dev_id;
	u32 ints = readl(kpd->base + KPD_INT_MASK_STS);
	u32 keys = readl(kpd->base + KPD_KEY_STATUS);
	int n;

	writel(KPD_INT_ALL, kpd->base + KPD_INT_CLR);
	if (!(ints & KPD_INT_ALL))
		return IRQ_NONE;

	for (n = 0; n < KPD_SLOTS; n++) {
		u8 slot = keys >> (8 * n);

		if (ints & KPD_INT_PRESS(n))
			sc8830_kpd_report(kpd, slot, 1);
		if (ints & KPD_INT_RELEASE(n))
			sc8830_kpd_report(kpd, slot, 0);
	}

	return IRQ_HANDLED;
}

/* Rows and columns to scan: those used by linux,keymap */
static int sc8830_kpd_lines(struct device *dev, u32 *rows, u32 *cols)
{
	u32 *map;
	int n, i, ret;

	n = device_property_count_u32(dev, "linux,keymap");
	if (n <= 0)
		return n ?: -EINVAL;
	map = kcalloc(n, sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;
	ret = device_property_read_u32_array(dev, "linux,keymap", map, n);
	if (!ret) {
		*rows = *cols = 0;
		for (i = 0; i < n; i++) {
			*rows |= BIT(KEY_ROW(map[i]));
			*cols |= BIT(KEY_COL(map[i]));
		}
	}
	kfree(map);
	return ret;
}

static void sc8830_kpd_aon_init(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *aon;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "aon-apb");
	if (!res)
		return;
	/* shared with other blocks, so no request_mem_region */
	aon = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!aon)
		return;
	writel(readl(aon + AON_APB_EB0) | AON_APB_KPD_EB, aon + AON_APB_EB0);
	writel(readl(aon + AON_APB_RTC_EB) | AON_APB_KPD_RTC_EB,
	       aon + AON_APB_RTC_EB);
	/* the bootloader leaves the keypad scanning, start from reset */
	writel(readl(aon + AON_APB_RST0) | AON_APB_KPD_RST, aon + AON_APB_RST0);
	usleep_range(2000, 3000);
	writel(readl(aon + AON_APB_RST0) & ~AON_APB_KPD_RST, aon + AON_APB_RST0);
}

static void sc8830_kpd_stop(void *data)
{
	struct sc8830_kpd *kpd = data;

	writel(0, kpd->base + KPD_INT_EN);
	writel(0, kpd->base + KPD_CTRL);
}

static int sc8830_kpd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc8830_kpd *kpd;
	u32 row_mask, col_mask;
	int irq, ret;

	kpd = devm_kzalloc(dev, sizeof(*kpd), GFP_KERNEL);
	if (!kpd)
		return -ENOMEM;

	kpd->base = devm_platform_ioremap_resource_byname(pdev, "kpd");
	if (IS_ERR(kpd->base))
		return PTR_ERR(kpd->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = matrix_keypad_parse_properties(dev, &kpd->rows, &kpd->cols);
	if (ret)
		return ret;
	if (kpd->rows > KPD_MAX_LINES || kpd->cols > KPD_MAX_LINES)
		return dev_err_probe(dev, -EINVAL,
				     "at most %d rows and columns\n",
				     KPD_MAX_LINES);
	kpd->row_shift = get_count_order(kpd->cols);

	ret = sc8830_kpd_lines(dev, &row_mask, &col_mask);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read linux,keymap\n");

	kpd->input = devm_input_allocate_device(dev);
	if (!kpd->input)
		return -ENOMEM;
	kpd->input->name = "sc8830-keypad";
	kpd->input->phys = "sc8830-keypad/input0";
	kpd->input->id.bustype = BUS_HOST;

	ret = matrix_keypad_build_keymap(NULL, NULL, kpd->rows, kpd->cols,
					 NULL, kpd->input);
	if (ret)
		return dev_err_probe(dev, ret, "invalid linux,keymap\n");
	input_set_capability(kpd->input, EV_MSC, MSC_SCAN);
	if (device_property_read_bool(dev, "autorepeat"))
		__set_bit(EV_REP, kpd->input->evbit);

	sc8830_kpd_aon_init(pdev);
	writel(0, kpd->base + KPD_INT_EN);
	writel(KPD_INT_ALL, kpd->base + KPD_INT_CLR);
	writel(0xffff, kpd->base + KPD_POLARITY);
	writel(1, kpd->base + KPD_CLK_DIV_CNT);
	writel(0xc, kpd->base + KPD_LONG_KEY_CNT);
	writel(5, kpd->base + KPD_DEBOUNCE_CNT);
	writel(32767, kpd->base + KPD_SLEEP_CNT);	/* 1 s of 32 kHz */

	ret = devm_request_irq(dev, irq, sc8830_kpd_irq, 0, dev_name(dev), kpd);
	if (ret)
		return ret;

	/* the matrix keeps scanning on the 32 kHz clock while we sleep */
	if (device_property_read_bool(dev, "wakeup-source")) {
		ret = devm_device_init_wakeup(dev);
		if (!ret)
			ret = devm_pm_set_wake_irq(dev, irq);
		if (ret)
			return ret;
	}

	ret = input_register_device(kpd->input);
	if (ret)
		return ret;

	writel(KPD_INT_DOWNUP, kpd->base + KPD_INT_EN);
	writel(KPD_CTRL_EN | KPD_CTRL_SLEEP_EN | KPD_CTRL_ROWS(row_mask) |
	       KPD_CTRL_COLS(col_mask), kpd->base + KPD_CTRL);

	return devm_add_action_or_reset(dev, sc8830_kpd_stop, kpd);
}

static const struct of_device_id sc8830_kpd_of_match[] = {
	{ .compatible = "sprd,sc8830-keypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc8830_kpd_of_match);

static struct platform_driver sc8830_kpd_driver = {
	.probe = sc8830_kpd_probe,
	.driver = {
		.name = "sprd-sc8830-keypad",
		.of_match_table = sc8830_kpd_of_match,
	},
};
module_platform_driver(sc8830_kpd_driver);

MODULE_DESCRIPTION("Spreadtrum SC8830/SC7730 keypad driver");
MODULE_LICENSE("GPL");
