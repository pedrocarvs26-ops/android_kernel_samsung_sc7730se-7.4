// SPDX-License-Identifier: GPL-2.0
/*
 * Spreadtrum SC8830/SC7730 ("shark") INTC: wake-up gates in front of the GIC.
 *
 * Once every core sits in WFI the GIC clock stops, and only interrupts
 * enabled in one of the four INTC blocks wake the CPU again (vendor 3.10
 * kernel, arch/arm/mach-sc/irq.c, gic_arch_extn). Interrupts still reach
 * the GIC without this, so nothing breaks, but each one waits for the
 * next timer tick: eMMC reads went from 0.6 to 15 MB/s at 4 KiB and USB
 * network latency from ~110 ms to 0.7 ms once the gates were open.
 *
 * Stacked on the GIC: unmask opens the INTC bit and then the GIC, mask goes
 * the other way. INTC n covers SPI 32n..32n+31 with bit = SPI % 32; bits 0
 * and 1 of every block are not wired. The AON wake-up mux, which only
 * matters for deep sleep, is not handled.
 */

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <dt-bindings/interrupt-controller/arm-gic.h>

#define INTC_BLOCKS		4
#define INTC_IRQ_EN		0x08
#define INTC_IRQ_DIS		0x0c
#define INTC_FIQ_DIS		0x2c

static void __iomem *intc_base[INTC_BLOCKS];

static void sc8830_intc_gate(struct irq_data *d, u32 reg)
{
	unsigned int spi = irqd_to_hwirq(d);

	if (spi / 32 < INTC_BLOCKS && spi % 32 >= 2)
		writel_relaxed(BIT(spi % 32), intc_base[spi / 32] + reg);
}

static void sc8830_intc_unmask(struct irq_data *d)
{
	sc8830_intc_gate(d, INTC_IRQ_EN);
	irq_chip_unmask_parent(d);
}

static void sc8830_intc_mask(struct irq_data *d)
{
	irq_chip_mask_parent(d);
	sc8830_intc_gate(d, INTC_IRQ_DIS);
}

static struct irq_chip sc8830_intc_chip = {
	.name			= "SC8830-INTC",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= sc8830_intc_mask,
	.irq_unmask		= sc8830_intc_unmask,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_type		= irq_chip_set_type_parent,
#ifdef CONFIG_SMP
	.irq_set_affinity	= irq_chip_set_affinity_parent,
#endif
	.flags			= IRQCHIP_SKIP_SET_WAKE,
};

static int sc8830_intc_translate(struct irq_domain *d,
				 struct irq_fwspec *fwspec,
				 unsigned long *hwirq, unsigned int *type)
{
	if (!is_of_node(fwspec->fwnode) || fwspec->param_count != 3)
		return -EINVAL;
	/* PPIs and SGIs are per-CPU and never pass the INTC */
	if (fwspec->param[0] != GIC_SPI)
		return -EINVAL;

	*hwirq = fwspec->param[1];
	*type = fwspec->param[2] & IRQ_TYPE_SENSE_MASK;
	return 0;
}

static int sc8830_intc_alloc(struct irq_domain *domain, unsigned int irq,
			     unsigned int nr_irqs, void *data)
{
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	irq_hw_number_t hwirq;
	unsigned int type;
	int err, i;

	err = sc8830_intc_translate(domain, fwspec, &hwirq, &type);
	if (err)
		return err;

	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_hwirq_and_chip(domain, irq + i, hwirq + i,
					      &sc8830_intc_chip, NULL);

	parent_fwspec = *fwspec;
	parent_fwspec.fwnode = domain->parent->fwnode;
	return irq_domain_alloc_irqs_parent(domain, irq, nr_irqs,
					    &parent_fwspec);
}

static const struct irq_domain_ops sc8830_intc_domain_ops = {
	.translate	= sc8830_intc_translate,
	.alloc		= sc8830_intc_alloc,
	.free		= irq_domain_free_irqs_common,
};

static int __init sc8830_intc_init(struct device_node *node,
				   struct device_node *parent)
{
	struct irq_domain *parent_domain, *domain;
	int i;

	parent_domain = parent ? irq_find_host(parent) : NULL;
	if (!parent_domain) {
		pr_err("%pOF: no parent interrupt controller\n", node);
		return -ENXIO;
	}

	for (i = 0; i < INTC_BLOCKS; i++) {
		intc_base[i] = of_iomap(node, i);
		if (!intc_base[i]) {
			pr_err("%pOF: cannot map INTC%d\n", node, i);
			goto err_unmap;
		}
		/* Gate everything; unmask opens what is actually in use */
		writel_relaxed(~0U, intc_base[i] + INTC_IRQ_DIS);
		writel_relaxed(~0U, intc_base[i] + INTC_FIQ_DIS);
	}

	domain = irq_domain_create_hierarchy(parent_domain, 0, 0,
					     of_fwnode_handle(node),
					     &sc8830_intc_domain_ops, NULL);
	if (!domain)
		goto err_unmap;

	pr_info("%pOF: %d INTC blocks stacked on %pOF\n", node, INTC_BLOCKS, parent);
	return 0;

err_unmap:
	while (i--)
		iounmap(intc_base[i]);
	return -ENOMEM;
}
IRQCHIP_DECLARE(sc8830_intc, "sprd,sc8830-intc", sc8830_intc_init);
