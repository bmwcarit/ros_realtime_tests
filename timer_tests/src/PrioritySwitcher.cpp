/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "PrioritySwitcher.h"

#include <sched.h>
#include <sys/resource.h>

#define RT_PRIORITY 95
#define NORMAL_PRIORITY 0

PrioritySwitcher::PrioritySwitcher(bool useFifoScheduling) : pid((pid_t) 0), defaultPriority(-1), fifoScheduling(useFifoScheduling)
{
	saveDefault();
}

PrioritySwitcher::PrioritySwitcher(int pid, bool useFifoScheduling) : pid((pid_t) pid), defaultPriority(-1), fifoScheduling(useFifoScheduling)
{
	saveDefault();
}

int PrioritySwitcher::switchToRealtimePriority()
{
	struct sched_param schedParam;
	schedParam.sched_priority = RT_PRIORITY;
	int sched_policy = fifoScheduling ? SCHED_FIFO : SCHED_RR;
	return sched_setscheduler(pid, sched_policy, &schedParam);
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

