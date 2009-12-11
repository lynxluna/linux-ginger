#include <linux/module.h>

void start_perf_counter(void)
{
	unsigned int r1 = 0;
	asm("mrc p15, 0, %0, c9, c12, 0" : "=r" (r1));
	r1 |= 0x1; /* enable counters */
	asm("mcr p15, 0, %0, c9, c12, 0" : : "r" (r1));
	return;
}
EXPORT_SYMBOL(start_perf_counter);

void start_cycle_counter(void)
{
	unsigned int r2 = 0;
	r2 = 0x80000000; /* enable cycle counter only */
	asm("mcr p15, 0, %0, c9, c12, 1" : : "r" (r2));
	return;
}
EXPORT_SYMBOL(start_cycle_counter);

unsigned int cycle_count(void)
{
	unsigned int rd = 0;
	asm("mrc p15, 0, %0, c9, c13, 0" : "=r" (rd));
	return rd;
}
EXPORT_SYMBOL(cycle_count);


void stop_cycle_counter(void)
{
	unsigned int r3 = 0;
	r3 = 0x0; /* disable cycle counter */
	asm("mcr p15, 0, %0, c9, c12, 1" : : "r" (r3));
	return;
}
EXPORT_SYMBOL(stop_cycle_counter);

void stop_perf_counter(void)
{
	unsigned int r1 = 0;
	r1 = 0x0; /* disable counters */
	asm("mcr p15, 0, %0, c9, c12, 0" : : "r" (r1));
	return;
}
EXPORT_SYMBOL(stop_perf_counter);
