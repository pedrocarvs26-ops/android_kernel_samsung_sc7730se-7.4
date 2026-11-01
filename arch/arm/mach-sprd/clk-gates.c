// SPDX-License-Identifier: GPL-2.0
/*
 * A bank of 32 enable bits of the SC7730 as gate clocks, "#clock-cells =
 * <1>" with the bit number, e.g. the AP APB gates at 0x71300000 for the
 * "enable" clock that mainline sprd_serial wants for UART0 (bit 13).
 *
 * Other drivers in mach-sprd still set their bits in the same registers
 * directly (INTC, I2C1), so every gate is CLK_IGNORE_UNUSED: switching
 * off "unused" gates at late init would cut blocks that are in use.
 *
 * Bring-up glue until there is a clock driver for this SoC.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define SC7730_GATES	32

static DEFINE_SPINLOCK(sc7730_gates_lock);

static int sc7730_gates_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	const char *parent, *name;
	struct resource *res;
	void __iomem *reg;
	int i;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	/* shared with other drivers, so no request_mem_region */
	reg = devm_ioremap(dev, res->start, resource_size(res));
	if (!reg)
		return -ENOMEM;

	parent = of_clk_get_parent_name(dev->of_node, 0);
	data = devm_kzalloc(dev, struct_size(data, hws, SC7730_GATES),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num = SC7730_GATES;

	for (i = 0; i < SC7730_GATES; i++) {
		name = devm_kasprintf(dev, GFP_KERNEL, "%s-%d", dev_name(dev), i);
		if (!name)
			return -ENOMEM;
		data->hws[i] = devm_clk_hw_register_gate(dev, name, parent,
							 CLK_IGNORE_UNUSED, reg,
							 i, 0, &sc7730_gates_lock);
		if (IS_ERR(data->hws[i]))
			return PTR_ERR(data->hws[i]);
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static const struct of_device_id sc7730_gates_of_match[] = {
	{ .compatible = "sprd,sc7730-gates" },
	{ }
};

static struct platform_driver sc7730_gates_driver = {
	.probe = sc7730_gates_probe,
	.driver = {
		.name = "sc7730-gates",
		.of_match_table = sc7730_gates_of_match,
	},
};
builtin_platform_driver(sc7730_gates_driver);
