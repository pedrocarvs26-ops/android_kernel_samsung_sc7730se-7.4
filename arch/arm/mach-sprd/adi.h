/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MACH_SPRD_ADI_H
#define __MACH_SPRD_ADI_H

#include <linux/types.h>

/*
 * Registers of the SC2723 PMIC, as offsets into the ADI slave window
 * (0x40038000). Bring-up helpers until mainline spi-sprd-adi and a
 * proper SC2723 MFD driver take over.
 */
#define SC2723_WDG(x)	(0x040 + (x))	/* PMIC watchdog */
#define SC2723_GLB(x)	(0x800 + (x))	/* global registers */

int sprd_adi_read(u32 reg, u32 *val);
int sprd_adi_write(u32 reg, u32 val);
int sprd_adi_update(u32 reg, u32 clr, u32 set);

#endif
