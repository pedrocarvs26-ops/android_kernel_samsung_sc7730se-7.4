// SPDX-License-Identifier: GPL-2.0
/*
 * Boot progress stripes for Spreadtrum bring-up without serial console.
 *
 * The Spreadtrum bootloader leaves its splash screen running (MIPI DSI
 * video mode keeps scanning out) and passes the buffer address as
 * "lcd_base=<hex>". We take that buffer out of the linear map early,
 * then paint one full-width white band per boot stage reached. White is
 * used on purpose: it looks the same whatever the pixel channel order.
 *
 * Screen layout (800x1280 portrait, 32 bpp):
 *   stage n  -> band at y = 64 + n * 112, 80 px high
 *   panic    -> checkerboard over the bottom 160 lines
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/notifier.h>
#include <linux/panic_notifier.h>
#include <linux/sprd-bootstripes.h>

#define FB_W		800
#define FB_H		1280
#define FB_SIZE		(FB_W * FB_H * 4)
#define BAND_Y(n)	(64 + (n) * 112)
#define BAND_H		80

/* Fallback: buffer reserved for the display by the vendor device tree */
#define DT_LCD_BASE	0x9ea44000
static phys_addr_t lcd_base = DT_LCD_BASE;
static u32 __iomem *fb;

/*
 * early_param handlers run from setup_arch() before paging_init(), so
 * the buffer can still be removed from the linear map here. That is
 * required: ARM refuses ioremap() of RAM that is linearly mapped, and the
 * kernel must never hand out the pages the display is scanning out.
 * The device tree reserves the usual buffer (no-map) itself; reserving it
 * here as well makes its reserved-memory node fail, so only a buffer at
 * another address is taken out here.
 */
static int __init sprd_lcd_base_setup(char *s)
{
	unsigned long v;

	if (!s || kstrtoul(s, 16, &v) || !v)
		return 0;
	lcd_base = v;
	if (lcd_base == DT_LCD_BASE)
		return 0;
	memblock_reserve(lcd_base, PAGE_ALIGN(FB_SIZE));
	memblock_mark_nomap(lcd_base, PAGE_ALIGN(FB_SIZE));
	return 0;
}
early_param("lcd_base", sprd_lcd_base_setup);

static void fill_rows(int y0, int h, u32 pat_a, u32 pat_b)
{
	int x, y;

	for (y = y0; y < y0 + h && y < FB_H; y++)
		for (x = 0; x < FB_W; x++)
			writel_relaxed(((x / 40 + y / 40) & 1) ? pat_b : pat_a,
				       fb + y * FB_W + x);
	wmb();
}

static bool sprd_fb_map(void)
{
	if (fb)
		return true;
	fb = ioremap_wc(lcd_base, FB_SIZE);
	if (!fb)
		return false;
	fill_rows(0, FB_H, 0, 0);		/* clear the splash screen */
	return true;
}

void sprd_bootstripe(enum sprd_bootstripe_stage stage)
{
	if (!sprd_fb_map())
		return;
	fill_rows(BAND_Y(stage), BAND_H, ~0U, ~0U);
}

void sprd_bootstripe_timer_irq(void)
{
	static bool seen;

	if (seen)
		return;
	seen = true;
	sprd_bootstripe(SPRD_STRIPE_TIMER_IRQ);
}

static int sprd_bootstripe_panic(struct notifier_block *nb,
				 unsigned long event, void *unused)
{
	if (sprd_fb_map())
		fill_rows(FB_H - 160, 160, ~0U, 0);
	return NOTIFY_DONE;
}

static struct notifier_block sprd_bootstripe_panic_nb = {
	.notifier_call = sprd_bootstripe_panic,
};

static int __init sprd_bootstripe_panic_init(void)
{
	atomic_notifier_chain_register(&panic_notifier_list,
				       &sprd_bootstripe_panic_nb);
	return 0;
}
early_initcall(sprd_bootstripe_panic_init);
