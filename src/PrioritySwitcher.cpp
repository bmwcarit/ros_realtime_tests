#include "PrioritySwitcher.h"
#include "TestParams.h"

#include <sched.h>
#include <sys/resource.h>

#define RT_PRIORITY 95
#define NORMAL_PRIORITY 0

PrioritySwitcher::PrioritySwitcher() : pid((pid_t) 0), defaultPriority(-1)
{
	saveDefault();
}

PrioritySwitcher::PrioritySwitcher(int pid) : pid((pid_t) pid), defaultPriority(-1)
{
	saveDefault();
}

int PrioritySwitcher::switchToRealtimePriority()
{
	struct sched_param schedParam;
	schedParam.sched_priority = RT_PRIORITY;
	return sched_setscheduler(pid, SCHED_FIFO, &schedParam);
}

int PrioritySwitcher::switchToNormalPriority()
{
	struct sched_param schedParam;
	schedParam.sched_priority = NORMAL_PRIORITY;
	int rc = sched_setscheduler(pid, defaultSchedulerPolicy, &schedParam);
	rc += setpriority(PRIO_PROCESS, (int) pid, defaultPriority);
}

void PrioritySwitcher::saveDefault()
{
	defaultPriority = getpriority(PRIO_PROCESS, (int) pid);
	defaultSchedulerPolicy = sched_getscheduler((int) pid);
}

PrioritySwitcher::~PrioritySwitcher()
{
}

