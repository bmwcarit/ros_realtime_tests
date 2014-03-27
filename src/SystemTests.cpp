#include "Logger.h"
#include "TestParams.h"
#include "main.h"

#include <gtest/gtest.h>

TEST(SystemTest, SystemClockPrecisionOfAtLeast1MicroSecond)
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

TEST(SystemTest, CanSwitchTestnodeToRealtimePriority)
{
	ASSERT_EQ(0, testnodePrioritySwitcher->switchToRealtimePriority());
}

TEST(SystemTest, CanSwitchTestnodeBackToNormalPriority)
{
	ASSERT_EQ(0, testnodePrioritySwitcher->switchToNormalPriority());
}

TEST(SystemTest, CanSwitchRoscoreToRealtimePriority)
{
	ASSERT_EQ(0, roscorePrioritySwitcher->switchToRealtimePriority());
}

TEST(SystemTest, CanSwitchRoscoreBackToNormalPriority)
{
	ASSERT_EQ(0, roscorePrioritySwitcher->switchToNormalPriority());
}

