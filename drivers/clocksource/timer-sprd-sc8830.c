// SPDX-License-Identifier: GPL-2.0
/*
 * Spreadtrum SC8830/SC7730 ("scx35") 32-bit general purpose timer.
 *
 * Register layout and programming sequence taken from the vendor
 * 3.10 kernel (arch/arm/mach-sc/timer_sc8830.c). Each timer block has
 * three channels with a stride of 0x20:
 *
 *   +0x00 LOAD    reload value, counts down
 *   +0x04 VALUE   current value
 *   +0x08 CTL     bit6 period mode, bit7 enable, bit8 "new" mode
 *   +0x0c INT     bit0 enable, bit3 clear, bit4 busy
 *   +0x10 CNT_RD  shadow counter, safe to read while running
 *
 * Channel 0 of the always-on (AON) timer is used as clockevent and
 * channel 1 of the same block as free running clocksource, exactly as
 * the vendor kernel does for CPU0. Both run from the 32.768 kHz clock.
 * On SC7730 the architected timer takes over once the system counter
 * runs; this one stays as a fallback.
 */

#include <linux/bitops.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/sprd-bootstripes.h>

#define SC8830_CH(n)		((n) * 0x20)
#define SC8830_LOAD		0x00
#define SC8830_VALUE		0x04
#define SC8830_CTL		0x08
#define SC8830_CTL_PERIOD	BIT(6)
#define SC8830_CTL_ENABLE	BIT(7)
#define SC8830_CTL_NEW		BIT(8)
#define SC8830_INT		0x0c
#define SC8830_INT_EN		BIT(0)
#define SC8830_INT_CLR		BIT(3)
#define SC8830_INT_BUSY		BIT(4)
#define SC8830_CNT_RD		0x10

#define EVT_CH			0
#define SRC_CH			1

/* AON APB clock gates (REG_AON_APB_APB_EB0) */
#define AON_APB_EB0		0x0
#define AON_EB0_AP_TMR0		BIT(12)
#define AON_EB0_AON_TMR		BIT(11)
#define AON_EB0_AP_SYST		BIT(10)

static void __iomem *evt_base;
static void __iomem *src_base;
static u32 sc8830_rate;

static inline void sc8830_ctl(void __iomem *ch, u32 val)
{
	writel_relaxed(val, ch + SC8830_CTL);
}

static int sc8830_set_next_event(unsigned long cycles,
				 struct clock_event_device *ce)
{
	while (readl_relaxed(evt_base + SC8830_INT) & SC8830_INT_BUSY)
		cpu_relax();
	sc8830_ctl(evt_base, 0);
	writel_relaxed(cycles, evt_base + SC8830_LOAD);
	sc8830_ctl(evt_base, SC8830_CTL_ENABLE);
	return 0;
}

static int sc8830_set_periodic(struct clock_event_device *ce)
{
	sc8830_ctl(evt_base, 0);
	writel_relaxed(DIV_ROUND_CLOSEST(sc8830_rate, HZ),
		       evt_base + SC8830_LOAD);
	sc8830_ctl(evt_base, SC8830_CTL_ENABLE | SC8830_CTL_PERIOD);
	writel_relaxed(SC8830_INT_EN, evt_base + SC8830_INT);
	return 0;
}

static int sc8830_set_oneshot(struct clock_event_device *ce)
{
	sc8830_ctl(evt_base, 0);
	writel_relaxed(SC8830_INT_EN, evt_base + SC8830_INT);
	return 0;
}

static int sc8830_shutdown(struct clock_event_device *ce)
{
	sc8830_ctl(evt_base, 0);
	writel_relaxed(SC8830_INT_CLR, evt_base + SC8830_INT);
	return 0;
}

static struct clock_event_device sc8830_clockevent = {
	.name			= "sc8830-aon-timer0",
	.features		= CLOCK_EVT_FEAT_PERIODIC |
				  CLOCK_EVT_FEAT_ONESHOT,
	.rating			= 300,
	.set_next_event		= sc8830_set_next_event,
	.set_state_periodic	= sc8830_set_periodic,
	.set_state_oneshot	= sc8830_set_oneshot,
	.set_state_shutdown	= sc8830_shutdown,
	.tick_resume		= sc8830_shutdown,
};

static irqreturn_t sc8830_timer_irq(int irq, void *dev_id)
{
	struct clock_event_device *ce = dev_id;
	u32 val = readl_relaxed(evt_base + SC8830_INT);

	writel_relaxed(val | SC8830_INT_CLR, evt_base + SC8830_INT);
	sprd_bootstripe_timer_irq();
	ce->event_handler(ce);
	return IRQ_HANDLED;
}

static u64 notrace sc8830_sched_read(void)
{
	return ~readl_relaxed(src_base + SC8830_CNT_RD);
}

static u64 sc8830_cs_read(struct clocksource *cs)
{
	return ~readl_relaxed(src_base + SC8830_CNT_RD);
}

static struct clocksource sc8830_clocksource = {
	.name	= "sc8830-aon-timer1",
	.rating	= 300,
	.read	= sc8830_cs_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init sc8830_set_bits(struct device_node *np, const char *name,
				   u32 off, u32 bits)
{
	int idx = of_property_match_string(np, "reg-names", name);
	void __iomem *base;

	if (idx < 0)
		return;
	base = of_iomap(np, idx);
	if (!base)
		return;
	writel_relaxed(readl_relaxed(base + off) | bits, base + off);
	iounmap(base);
}

static int __init sc8830_timer_init(struct device_node *np)
{
	int irq, ret, idx;

	if (of_property_read_u32(np, "clock-frequency", &sc8830_rate))
		sc8830_rate = 32768;

	/* Clock gates for AON timer, AP timer 0 and sys counter */
	sc8830_set_bits(np, "aon-apb", AON_APB_EB0, AON_EB0_AON_TMR |
			AON_EB0_AP_TMR0 | AON_EB0_AP_SYST);

	idx = of_property_match_string(np, "reg-names", "timer");
	src_base = of_iomap(np, idx < 0 ? 0 : idx);
	if (!src_base) {
		pr_err("sc8830-timer: cannot map registers\n");
		return -ENXIO;
	}
	evt_base = src_base + SC8830_CH(EVT_CH);
	src_base = src_base + SC8830_CH(SRC_CH);

	/* Same reset sequence as the vendor kernel */
	sc8830_ctl(evt_base, 0);
	writel_relaxed(SC8830_INT_CLR, evt_base + SC8830_INT);

	writel_relaxed(0, src_base + SC8830_INT);
	sc8830_ctl(src_base, SC8830_CTL_NEW | SC8830_CTL_PERIOD);
	writel_relaxed(~0U, src_base + SC8830_LOAD);
	sc8830_ctl(src_base, SC8830_CTL_NEW | SC8830_CTL_PERIOD);
	sc8830_ctl(src_base, SC8830_CTL_ENABLE | SC8830_CTL_NEW |
		   SC8830_CTL_PERIOD);

	sched_clock_register(sc8830_sched_read, 32, sc8830_rate);
	ret = clocksource_register_hz(&sc8830_clocksource, sc8830_rate);
	if (ret)
		pr_err("sc8830-timer: clocksource registration failed\n");

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("sc8830-timer: no interrupt\n");
		return -EINVAL;
	}

	ret = request_irq(irq, sc8830_timer_irq, IRQF_TIMER | IRQF_IRQPOLL,
			  "sc8830-timer", &sc8830_clockevent);
	if (ret) {
		pr_err("sc8830-timer: cannot request irq %d\n", irq);
		return ret;
	}

	sc8830_clockevent.cpumask = cpu_possible_mask;
	sc8830_clockevent.irq = irq;
	clockevents_config_and_register(&sc8830_clockevent, sc8830_rate,
					2, 0xf0000000);

	pr_info("sc8830-timer: %u Hz, clockevent irq %d\n", sc8830_rate, irq);
	return 0;
}
TIMER_OF_DECLARE(sc8830_timer, "sprd,sc8830-timer", sc8830_timer_init);
