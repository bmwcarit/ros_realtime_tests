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
