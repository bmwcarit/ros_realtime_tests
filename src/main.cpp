#include "main.h"
#include "Logger.h"

#include "ros/ros.h"
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
	std::stringstream ss;
	ss << "Running " << loops << " measurements per testcase.";
	Logger::INFO(ss.str());
	int x = 1;
	char* y[1];
	y[0] = argv[0];
	ros::init(x, y, "Timer_tests");
	nodeHandle = new ros::NodeHandle;
	testing::InitGoogleTest(&x, y);
	return RUN_ALL_TESTS();
}

