#include "main.h"
#include "Logger.h"

#include <gtest/gtest.h>

ros::NodeHandle* nodeHandle;
int loops;

int main(int argc, char* argv[])
{
	if(argc != 2)
	{
		Logger::ERROR("Usage: timer_tests <measurements_per_testcase>");
		return 1;
	}
	loops = atoi(argv[1]);
	int x = 1;
	char* y[1];
	y[0] = (char*) "timer_tests";
	ros::init(x, y, "Timer_tests");
	testing::InitGoogleTest(&x, y);
	nodeHandle = new ros::NodeHandle;
	return RUN_ALL_TESTS();
}

