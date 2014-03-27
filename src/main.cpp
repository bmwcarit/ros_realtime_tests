#include "main.h"
#include "Logger.h"

#include "ros/ros.h"
#include <gtest/gtest.h>

ros::NodeHandle* nodeHandle;
PrioritySwitcher* testnodePrioritySwitcher;
PrioritySwitcher* roscorePrioritySwitcher;
int loops;

int main(int argc, char* argv[])
{	
	testnodePrioritySwitcher = new PrioritySwitcher();
	if(argc != 3)
	{
		Logger::ERROR("Usage: timer_tests <measurements_per_testcase> <roscore_pid>");
		return -1;
	}
	loops = atoi(argv[1]);
	roscorePrioritySwitcher = new PrioritySwitcher(atoi(argv[2]));
	int x = 1;
	char* y[1];
	y[0] = argv[0];
	ros::init(x, y, "Timer_tests");
	nodeHandle = new ros::NodeHandle;
	testing::InitGoogleTest(&x, y);
	return RUN_ALL_TESTS();
}

