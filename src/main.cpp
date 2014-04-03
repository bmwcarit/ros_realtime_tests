#include "main.h"
#include "Logger.h"

#include "ros/ros.h"
#include <gtest/gtest.h>

ros::NodeHandle* nodeHandle;
PrioritySwitcher* testnodePrioritySwitcher;
PrioritySwitcher* roscorePrioritySwitcher;
int loops;
int timeout_us;
bool testnodeRT;
bool roscoreRT;
bool fifoScheduling;

void printUsage()
{
	Logger::ERROR("Usage: timer_tests <amount_measurements> <timeout_value_us> <roscore_pid> <testnode_rt 0/1> <roscore_rt 0/1> <rt_sched_policy 'FIFO'/'RR'>");
}

bool setSchedulingPolicy(std::string arg)
{
	if(arg.length() == 2)
	{
		if(strcasecmp(arg.c_str(), "RR") == 0)
		{
			fifoScheduling = false;
			return true;
		}
		return false;
	}
	if(arg.length() == 4)
	{
		if(strcasecmp(arg.c_str(), "FIFO") == 0)
		{
			fifoScheduling = true;
			return true;
		}
		return false;
	}
	return false;
}

bool setProcessPriorities()
{
	int rc = 0;
	if(testnodeRT)
	{
		rc += testnodePrioritySwitcher->switchToRealtimePriority();
	} else {
		rc += testnodePrioritySwitcher->switchToNormalPriority();
	}
	if(roscoreRT)
	{
		rc += roscorePrioritySwitcher->switchToRealtimePriority();
	} else {
		rc += roscorePrioritySwitcher->switchToNormalPriority();
	}
	return rc == 0;
}

int main(int argc, char* argv[])
{	
	if(argc != 7)
	{
		printUsage();
		return -1;
	}
	loops = atoi(argv[1]);
	timeout_us = atoi(argv[2]);
	testnodeRT = argv[4][0] == '1';
	roscoreRT = argv[5][0] == '1';
	if((argv[4][0] != '0' && argv[4][0] != '1') || (argv[5][0] != '0' && argv[5][0] != '1') || !setSchedulingPolicy(argv[6]))
	{
		printUsage();
		return -1;
	}
	testnodePrioritySwitcher = new PrioritySwitcher(0, fifoScheduling);
	roscorePrioritySwitcher = new PrioritySwitcher(atoi(argv[3]), fifoScheduling);
	int x = 1;
	char* y[1];
	y[0] = argv[0];
	if(!setProcessPriorities())
	{
		Logger::ERROR("Couldn't set priorities appropriately");
		return 1;
	}
	ros::init(x, y, "Timer_tests");
	nodeHandle = new ros::NodeHandle;
	testing::InitGoogleTest(&x, y);
	return RUN_ALL_TESTS();
}

