#include "ros/ros.h"
#include "Logger.h"

#include <gtest/gtest.h>
#include <time.h>
#include <unistd.h>

#define CLOCK_ID CLOCK_MONOTONIC_RAW

ros::NodeHandle* nodeHandle;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Timer_tests");
	testing::InitGoogleTest(&argc, argv);
	nodeHandle = new ros::NodeHandle;
	return RUN_ALL_TESTS();
}

TEST(SystemClockPrecision, PrecisionOfAtLeast1MicroSecond)
{
	struct timespec clockResolution;
	clock_getres(CLOCK_ID, &clockResolution);
	std::stringstream ss;
	ss << "System reports time measurement with resolution of ";
	ss << clockResolution.tv_nsec;
	ss << " nanoseconds";
	std::string infoMessage = ss.str();
	Logger::INFO(infoMessage);
	ASSERT_LE(clockResolution.tv_nsec, 1000);
}

struct timespec ts;
bool callbackCalled = false;

void timerCallback(const ros::TimerEvent&)
{
	clock_gettime(CLOCK_ID, &ts);
	callbackCalled = true;
}

TEST(RosOneShotTimer, LatencySmallerThan1MicroSecond)
{
	ros::Timer rosTimer = nodeHandle->createTimer(ros::Duration(0.1), &timerCallback);
	struct timespec start;
	clock_gettime(CLOCK_ID, &start);
	while(!callbackCalled)
	{
		ros::spinOnce();
	}
	long diff = ts.tv_nsec - start.tv_nsec - 100000000;

	std::stringstream ss;
	ss << "Measured latency of Oneshot timer: ";
	ss << diff << " nanoseconds";
	std::string infoMsg = ss.str();
	Logger::INFO(infoMsg);
	ASSERT_LE(diff, 1000);
}
