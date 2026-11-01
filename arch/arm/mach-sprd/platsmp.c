// SPDX-License-Identifier: GPL-2.0
/*
 * Secondary CPU bring-up for the Spreadtrum SC7730 (4x Cortex-A7).
 *
 * Cores 1-3 come out of reset into the boot ROM (high vectors), which
 * waits for their bit in the AP AHB HOLDING_PEN register and then jumps
 * to JMP_ADDR_CA7_Cn. They start in the Secure world in SVC mode. Power
 * comes from the PMU: PD_CA7_Cn_CFG with FORCE_SHUTDOWN clear powers a
 * core up, the state nibble of PWR_STATUS0_DBG reads 0 when it is on and
 * 7 when it is off. Sequence from the vendor 3.10 kernel
 * (arch/arm/mach-sc/platsmp.c), confirmed by booting core 1 into a probe
 * stub from user space. The vendor's software pen is not needed.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/spinlock.h>

#include <asm/cacheflush.h>
#include <asm/cp15.h>
#include <asm/smp_plat.h>

#include "sc7730.h"

#define AP_AHB_PHYS		0x20d00000
#define AHB_CA7_RST_SET		0x0008		/* bits 3:0 core reset */
#define AHB_HOLDING_PEN		0x0014		/* bit n releases core n */
#define AHB_JMP_ADDR(n)		(0x0018 + 4 * (n))
#define AHB_CA7_STANDBY		0x3034		/* bits 3:0 STANDBYWFI */

#define PMU_PHYS		0x402b0000
#define PMU_PD_CA7_C(n)		(0x0004 + 4 * (n))
#define PD_FORCE_SHUTDOWN	BIT(25)
#define PD_AUTO_SHUTDOWN_EN	BIT(24)
#define PD_ON_DELAYS		0x00040404	/* power-on/sequence/isolation delays */
#define PMU_PWR_STATUS0		0x00bc
#define PD_STATE(v, n)		(((v) >> (4 * ((n) + 1))) & 0xf)
#define PD_STATE_ON		0
#define PD_STATE_OFF		7

static void __iomem *ap_ahb, *pmu;
static DEFINE_SPINLOCK(sc7730_smp_lock);

static unsigned int sc7730_core(unsigned int cpu)
{
	return MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 0);
}

static void __init sc7730_smp_prepare_cpus(unsigned int max_cpus)
{
	/* shared with other users of these blocks, no request_mem_region */
	ap_ahb = ioremap(AP_AHB_PHYS, 0x4000);
	pmu = ioremap(PMU_PHYS, 0x100);
	if (!ap_ahb || !pmu)
		pr_err("sc7730-smp: cannot map AP AHB/PMU\n");
}

static int sc7730_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned int n = sc7730_core(cpu);
	int i;

	if (!ap_ahb || !pmu || n < 1 || n > 3)
		return -ENODEV;

	spin_lock(&sc7730_smp_lock);
	/*
	 * The ROM jumps with interworking, and secondary_startup_arm must be
	 * entered in ARM state. The LLVM assembler marks it as a Thumb
	 * function in a Thumb-2 kernel (its ENDPROC comes after the switch
	 * to Thumb), so the address would carry bit 0 and the core would
	 * start decoding ARM code as Thumb.
	 */
	writel(__pa_symbol(secondary_startup_arm) & ~1UL,
	       ap_ahb + AHB_JMP_ADDR(n));
	writel(readl(ap_ahb + AHB_HOLDING_PEN) | BIT(n),
	       ap_ahb + AHB_HOLDING_PEN);
	writel(readl(ap_ahb + AHB_CA7_RST_SET) | BIT(n),
	       ap_ahb + AHB_CA7_RST_SET);
	/* FORCE_SHUTDOWN clear powers the core up; AUTO_SHUTDOWN stays off */
	writel(PD_ON_DELAYS, pmu + PMU_PD_CA7_C(n));
	for (i = 0; i < 50; i++) {
		if (PD_STATE(readl(pmu + PMU_PWR_STATUS0), n) == PD_STATE_ON)
			break;
		udelay(100);
	}
	writel(readl(ap_ahb + AHB_CA7_RST_SET) & ~BIT(n),
	       ap_ahb + AHB_CA7_RST_SET);
	spin_unlock(&sc7730_smp_lock);

	/* the ROM pen may wait in WFE */
	dsb_sev();
	return 0;
}

static void sc7730_secondary_init(unsigned int cpu)
{
	sc7730_set_cntfrq();
}

#ifdef CONFIG_HOTPLUG_CPU
static void sc7730_cpu_die(unsigned int cpu)
{
	v7_exit_coherency_flush(louis);
	while (1)
		wfi();
}

static int sc7730_cpu_kill(unsigned int cpu)
{
	unsigned int n = sc7730_core(cpu);
	int i;

	/* never cut power to a core that has not reached WFI */
	for (i = 0; i < 100; i++) {
		if (readl(ap_ahb + AHB_CA7_STANDBY) & BIT(n))
			break;
		usleep_range(100, 200);
	}
	if (!(readl(ap_ahb + AHB_CA7_STANDBY) & BIT(n)))
		return 0;

	spin_lock(&sc7730_smp_lock);
	writel((readl(pmu + PMU_PD_CA7_C(n)) | PD_FORCE_SHUTDOWN) &
	       ~PD_AUTO_SHUTDOWN_EN, pmu + PMU_PD_CA7_C(n));
	spin_unlock(&sc7730_smp_lock);

	for (i = 0; i < 100; i++) {
		if (PD_STATE(readl(pmu + PMU_PWR_STATUS0), n) == PD_STATE_OFF)
			return 1;
		udelay(60);
	}
	return 0;
}
#endif

static const struct smp_operations sc7730_smp_ops __initconst = {
	.smp_prepare_cpus	= sc7730_smp_prepare_cpus,
	.smp_secondary_init	= sc7730_secondary_init,
	.smp_boot_secondary	= sc7730_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= sc7730_cpu_die,
	.cpu_kill		= sc7730_cpu_kill,
#endif
};
CPU_METHOD_OF_DECLARE(sc7730_smp, "sprd,sc7730-smp", &sc7730_smp_ops);
