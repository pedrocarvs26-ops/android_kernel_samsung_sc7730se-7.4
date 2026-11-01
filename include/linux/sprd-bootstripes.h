/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_SPRD_BOOTSTRIPES_H
#define _LINUX_SPRD_BOOTSTRIPES_H

/*
 * Boot progress stripes painted straight into the Spreadtrum bootloader
 * framebuffer. Bring-up debug aid, see CONFIG_SPRD_BOOTSTRIPES.
 */
enum sprd_bootstripe_stage {
	SPRD_STRIPE_MM		= 1,	/* mm_core_init() done, ioremap works */
	SPRD_STRIPE_TIME	= 2,	/* time_init() returned */
	SPRD_STRIPE_IRQ_ON	= 3,	/* interrupts enabled */
	SPRD_STRIPE_CONSOLE	= 4,	/* console_init() done */
	SPRD_STRIPE_CALIBRATE	= 5,	/* calibrate_delay() done */
	SPRD_STRIPE_KINIT	= 6,	/* kernel_init thread running */
	SPRD_STRIPE_EXEC_INIT	= 7,	/* about to exec /init */
	SPRD_STRIPE_TIMER_IRQ	= 8,	/* first timer interrupt seen */
};

#ifdef CONFIG_SPRD_BOOTSTRIPES
void sprd_bootstripe(enum sprd_bootstripe_stage stage);
void sprd_bootstripe_timer_irq(void);
#else
static inline void sprd_bootstripe(enum sprd_bootstripe_stage stage) { }
static inline void sprd_bootstripe_timer_irq(void) { }
#endif

#endif
