#include "Logger.h"
#include "TestParams.h"
#include "PrioritySwitcher.h"

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

TEST(SystemTest, CanSwitchToRealtimePriority)
{
	ASSERT_EQ(switchToRealtimePriority(), 0);
}

TEST(SystemTest, CanSwitchBackToNormalPriority)
{
	ASSERT_EQ(switchToNormalPriority(), 0);
}

