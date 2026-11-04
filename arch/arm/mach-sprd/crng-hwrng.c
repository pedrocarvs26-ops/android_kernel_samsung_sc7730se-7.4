// SPDX-License-Identifier: GPL-2.0
/*
 * /dev/hw_random for the SC7730, which has no hardware RNG the kernel knows
 * of (the vendor kernel has none either). Android's prng_seeder needs it:
 * init creates the daemon's socket already listening, and without
 * /dev/hw_random the daemon parks before accept(), so every process whose
 * FIPS BoringSSL asks the daemon for its first seed blocks in read() forever.
 *
 * Serve the kernel's CRNG instead; prng_seeder conditions the bytes with its
 * own DRBG anyway. Quality 1 (of 1024): the hwrng core feeds every rng back
 * into the pool, these bytes must not be credited as entropy (0 would mean
 * the default, full credit).
 */
#include <linux/hw_random.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/random.h>

static int sc7730_crng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	get_random_bytes(buf, max);
	return max;
}

static struct hwrng sc7730_crng = {
	.name = "sc7730-crng",
	.read = sc7730_crng_read,
	.quality = 1,
};

static int __init sc7730_crng_init(void)
{
	if (!of_machine_is_compatible("sprd,sc7730"))
		return 0;
	return hwrng_register(&sc7730_crng);
}
device_initcall(sc7730_crng_init);
