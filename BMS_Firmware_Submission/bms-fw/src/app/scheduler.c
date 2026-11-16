#include "app/scheduler.h"

#include <string.h>

typedef struct
{
    const char *name;
    SchedTaskFn fn;
    uint32_t period_ms;
    uint32_t next_due_ms;
    bool used;
} SchedTask_t;

static SchedTask_t s_tasks[SCHED_MAX_TASKS];
static uint32_t s_executions;
static uint32_t s_overruns;
static uint8_t s_task_count;

void Sched_Init(uint32_t now_ms)
{
    (void)memset(s_tasks, 0, sizeof(s_tasks));
    s_executions = 0U;
    s_overruns = 0U;
    s_task_count = 0U;
    (void)now_ms;
}

bool Sched_Register(const char *name, uint32_t period_ms, SchedTaskFn fn)
{
    if ((fn == NULL) || (period_ms == 0U) || (s_task_count >= SCHED_MAX_TASKS))
    {
        return false;
    }

    SchedTask_t *task = &s_tasks[s_task_count];
    task->name = name;
    task->fn = fn;
    task->period_ms = period_ms;
    task->next_due_ms = 0U;
    task->used = true;
    s_task_count++;
    return true;
}

void Sched_Run(uint32_t now_ms)
{
    for (uint8_t i = 0U; i < s_task_count; ++i)
    {
        SchedTask_t *task = &s_tasks[i];
        if (!task->used)
        {
            continue;
        }

        /* Unsigned arithmetic keeps the comparison correct across the 49.7 day
         * wrap of a 32-bit millisecond counter. */
        if ((now_ms - task->next_due_ms) < task->period_ms)
        {
            continue;
        }

        task->next_due_ms += task->period_ms;

        /* If the task is more than one period late, re-base instead of running
         * it several times in a row.  This bounds the work done in one tick. */
        if ((now_ms - task->next_due_ms) >= task->period_ms)
        {
            task->next_due_ms = now_ms + task->period_ms;
            s_overruns++;
        }

        task->fn(now_ms);
        s_executions++;
    }
}

uint32_t Sched_GetExecutions(void)
{
    return s_executions;
}

uint32_t Sched_GetOverruns(void)
{
    return s_overruns;
}

uint8_t Sched_GetTaskCount(void)
{
    return s_task_count;
}
