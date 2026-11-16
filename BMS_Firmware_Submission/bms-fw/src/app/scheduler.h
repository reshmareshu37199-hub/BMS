#ifndef BMS_APP_SCHEDULER_H
#define BMS_APP_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#define SCHED_MAX_TASKS 8U

typedef void (*SchedTaskFn)(uint32_t now_ms);

/**
 * Cooperative, timer-driven super-loop scheduler.
 *
 * The scheduler is deliberately tiny: a fixed table of periodic tasks, each
 * with a period in milliseconds and a "next due" timestamp.  Sched_Run() is
 * called from the 1 ms tick (a timer interrupt or the simulation loop) and
 * executes every task whose deadline has passed.
 *
 * Design notes:
 *  - The task table is static, so there is no allocation and no unbounded
 *    search.  Worst case is SCHED_MAX_TASKS comparisons per tick.
 *  - A task that overruns its period is not executed repeatedly to "catch up".
 *    Instead the deadline is re-based and an overrun counter is incremented.
 *    This keeps a slow task from monopolising the CPU after a long interrupt.
 *  - Tasks must be short and non-blocking.  Periods are chosen so that the
 *    expected load is far below 100 %; Appendix A of docs/DESIGN.md contains
 *    the timing budget.
 */
void Sched_Init(uint32_t now_ms);

/** Register a task.  Returns false when the table is full or args are invalid. */
bool Sched_Register(const char *name, uint32_t period_ms, SchedTaskFn fn);

/** Run all tasks that are due at now_ms.  Bounded by SCHED_MAX_TASKS. */
void Sched_Run(uint32_t now_ms);

uint32_t Sched_GetExecutions(void);
uint32_t Sched_GetOverruns(void);
uint8_t Sched_GetTaskCount(void);

#endif /* BMS_APP_SCHEDULER_H */
