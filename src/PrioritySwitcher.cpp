#include "TestParams.h"

#include <sched.h>

#define RT_PRIORITY 99
#define NORMAL_PRIORITY 0

const int normalSchedulerPolicy = sched_getscheduler(0);

int switchToRealtimePriority()
{
	struct sched_param schedParam;
	schedParam.sched_priority = RT_PRIORITY;
	return sched_setscheduler(0, SCHED_FIFO, &schedParam);
}

int switchToNormalPriority()
{
	struct sched_param schedParam;
	schedParam.sched_priority = NORMAL_PRIORITY;
	return sched_setscheduler(0, normalSchedulerPolicy, &schedParam);
}

