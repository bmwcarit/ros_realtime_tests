#include "main.h"
#include "Logger.h"

#include <gtest/gtest.h>

ros::NodeHandle* nodeHandle;
int loops = 1000000;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Timer_tests");
	testing::InitGoogleTest(&argc, argv);
	nodeHandle = new ros::NodeHandle;
	return RUN_ALL_TESTS();
}

