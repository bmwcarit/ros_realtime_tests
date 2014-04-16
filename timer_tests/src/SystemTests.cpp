/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "main.h"
#include "OneShotLatencyMeasurer.h"
#include <gtest/gtest.h>
#include <rt_tests_support/Logger.h>

TEST(SystemTest, SystemClockPrecisionOfAtLeast1MicroSecond)
{
	struct timespec clockResolution;
	clock_getres(OneShotLatencyMeasurer::clock_id, &clockResolution);
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

