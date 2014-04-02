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

void printUsage()
{
	Logger::ERROR("Usage: timer_tests <measurements_per_testcase> <timeout_value_us> <roscore_pid> <testnode_rt 0/1> <roscore_rt 0/1>");
}

int main(int argc, char* argv[])
{	
	testnodePrioritySwitcher = new PrioritySwitcher();
	if(argc != 6)
	{
		printUsage();
		return -1;
	}
	loops = atoi(argv[1]);
	timeout_us = atoi(argv[2]);
	roscorePrioritySwitcher = new PrioritySwitcher(atoi(argv[3]));
	testnodeRT = argv[4][0] == '1';
	roscoreRT = argv[5][0] == '1';
	if((argv[4][0] != '0' && argv[4][0] != '1') || (argv[5][0] != '0' && argv[5][0] != '1'))
	{
		printUsage();
		return -1;
	}
	int x = 1;
	char* y[1];
	y[0] = argv[0];
	ros::init(x, y, "Timer_tests");
	nodeHandle = new ros::NodeHandle;
	testing::InitGoogleTest(&x, y);
	return RUN_ALL_TESTS();
}

