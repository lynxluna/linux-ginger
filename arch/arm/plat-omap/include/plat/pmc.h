#ifndef __PMC_H__
#define __PMC_H__
extern void start_perf_counter(void);
extern void start_cycle_counter(void);
extern void stop_cycle_counter(void);
extern void stop_perf_counter(void);
extern unsigned int cycle_count(void);
extern unsigned int delay_sram;
#endif
