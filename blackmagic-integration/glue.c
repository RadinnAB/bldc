#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <adiv5.h>

const bool connect_assert_srst = false;

bool parse_enable_or_disable(const char *s, bool *out) {
	*out = (s[0] == 'e');
	return true;
}

void gdb_outf(const char *fmt, ...)
{
	(void)fmt;
}

bool cortexa_probe(ADIv5_AP_t *apb, uint32_t debug_base)
{
	(void)apb;
	(void)debug_base;
	return false;
}

void rp_rescue_probe(ADIv5_AP_t *ap)
{
	(void)ap;
}

void efm32_aap_probe(ADIv5_AP_t *ap)
{
	(void)ap;
}

void kinetis_mdm_probe(ADIv5_AP_t *ap)
{
	(void)ap;
}

bool no_target_probe(void *t)
{
	(void)t;
	return false;
}

/* These decalartions allows us to remove a lot of blackmagic files from the
   build without performing surgery on the sources. These can be completely
   optimized away by enabling LTO. */
__attribute__ ((weak, alias("no_target_probe"))) bool gd32f1_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool stm32f1_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool stm32f4_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool stm32h7_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool stm32l0_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool stm32l1_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool stm32l4_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool stm32g0_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool lmi_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool lpc11xx_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool lpc15xx_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool lpc17xx_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool lpc43xx_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool lpc546xx_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool sam3x_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool sam4l_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool nrf51_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool samd_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool samx5x_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool kinetis_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool efm32_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool msp432_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool ke04_probe(void *t);
__attribute__ ((weak, alias("no_target_probe"))) bool rp_probe(void *t);
