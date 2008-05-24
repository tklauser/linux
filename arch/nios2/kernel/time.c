#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/profile.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/clocksource.h>

#include <asm/segment.h>
#include <asm/io.h>
#include <asm/nios.h>

#define	TICK_SIZE (tick_nsec / 1000)
#define NIOS2_TIMER_PERIOD (nasys_clock_freq/HZ)

static unsigned long nios2_timer_count;

static inline int set_rtc_mmss(unsigned long nowtime)
{
  return 0;
}

/* Timer timeout status */
#define nios2_timer_TO	(inw(&na_timer0->np_timerstatus) & np_timerstatus_to_mask)

static inline unsigned long read_timersnapshot(void)
{
	unsigned long count;

	outw(0, &na_timer0->np_timersnapl);
	count = inw(&na_timer0->np_timersnaph) << 16 | inw(&na_timer0->np_timersnapl);

	return count;
}

static inline void write_timerperiod(unsigned long period)
{
	na_timer0->np_timerperiodl = period;
	na_timer0->np_timerperiodh = period >> 16;
}

/*
 * timer_interrupt() needs to keep up the real-time clock,
 * as well as call the "do_timer()" routine every clocktick
 */
irqreturn_t timer_interrupt(int irq, void *dummy)
{
	/* last time the cmos clock got updated */
	static long last_rtc_update=0;

	na_timer0->np_timerstatus = 0; /* Clear the interrupt condition */
	nios2_timer_count += NIOS2_TIMER_PERIOD;

	write_seqlock(&xtime_lock);

	do_timer(1);
	profile_tick(CPU_PROFILING);
	/*
	 * If we have an externally synchronized Linux clock, then update
	 * CMOS clock accordingly every ~11 minutes. Set_rtc_mmss() has to be
	 * called as close as possible to 500 ms before the new second starts.
	 */
	if (ntp_synced() &&
	    xtime.tv_sec > last_rtc_update + 660 &&
	    (xtime.tv_nsec / 1000) >= 500000 - ((unsigned) TICK_SIZE) / 2 &&
	    (xtime.tv_nsec  / 1000) <= 500000 + ((unsigned) TICK_SIZE) / 2) {
	  if (set_rtc_mmss(xtime.tv_sec) == 0)
	    last_rtc_update = xtime.tv_sec;
	  else
	    last_rtc_update = xtime.tv_sec - 600; /* do it again in 60 s */
	}

	write_sequnlock(&xtime_lock);

#ifndef CONFIG_SMP
	update_process_times(user_mode(get_irq_regs()));
#endif

	return(IRQ_HANDLED);
}

static cycle_t nios2_timer_read(void)
{
	unsigned long flags;
	u32 cycles;
	u32 tcn;

	local_irq_save(flags);
	tcn = NIOS2_TIMER_PERIOD - 1 - read_timersnapshot();
	cycles = nios2_timer_count;
	local_irq_restore(flags);

	return cycles + tcn;
}

static struct clocksource nios2_timer = {
	.name	= "timer",
	.rating	= 250,
	.read	= nios2_timer_read,
	.shift	= 20,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct irqaction nios2_timer_irq = {
	.name	 = "timer",
	.flags	 = IRQF_DISABLED | IRQF_TIMER,
	.handler = timer_interrupt,
};

void __init time_init(void)
{
	unsigned int year, mon, day, hour, min, sec;
	extern void arch_gettod(int *year, int *mon, int *day, int *hour,
				int *min, int *sec);
	
	arch_gettod(&year, &mon, &day, &hour, &min, &sec);

	if ((year += 1900) < 1970)
		year += 100;
	xtime.tv_sec = mktime(year, mon, day, hour, min, sec);
	xtime.tv_nsec = 0;
	wall_to_monotonic.tv_sec = -xtime.tv_sec;

	/* dont use request_irq here, kmalloc is not ready yet */
	setup_irq(na_timer0_irq, &nios2_timer_irq);
	write_timerperiod(NIOS2_TIMER_PERIOD-1);

	/* clocksource initialize */
	nios2_timer.mult = clocksource_hz2mult(nasys_clock_freq, nios2_timer.shift);
	clocksource_register(&nios2_timer);

	/* interrupt enable + continuous + start */
	na_timer0->np_timercontrol = np_timercontrol_start_mask
				   + np_timercontrol_cont_mask
				   + np_timercontrol_ito_mask;
}
