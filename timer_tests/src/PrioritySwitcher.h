/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef PRIORITYSWITCHER_H_
#define PRIORITYSWITCHER_H_

#include <sched.h>
#include <sys/resource.h>

class PrioritySwitcher {
public:
	PrioritySwitcher(bool useFifoScheduling);
	PrioritySwitcher(int pid, bool useFifoScheduling);
	~PrioritySwitcher();

	//If no argument provided the priority of the current process is modified
	int switchToRealtimePriority();
	int switchToNormalPriority();
private:
	pid_t pid;
	pid_t defaultPriority;
	int defaultSchedulerPolicy;
	bool fifoScheduling;

	void saveDefault();
};

#endif //PRIORITYSWITCHER_H_
