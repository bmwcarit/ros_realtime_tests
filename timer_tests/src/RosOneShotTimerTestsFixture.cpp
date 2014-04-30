/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Config.h"
#include "OneShotLatencyMeasurer.h"
#include "RosOneShotTimerTestsFixture.h"
#include <math.h>
#include <rt_tests_support/Logger.h>

MeasurementDataEvaluator* RosOneShotTimerTests::measurementData;
bool RosOneShotTimerTests::setupSucceeded;

void RosOneShotTimerTests::SetUpTestCase()
{
	Logger::INFO("Performing ROS Timer latency measurements...");
	setupSucceeded = false;
	OneShotLatencyMeasurer measurer(Config::getConfig()->loops, Config::getConfig()->timeout_us*1000, Config::getConfig()->nodeHandle, Config::getConfig()->testnodeRT);
	measurer.measure();
	measurementData = measurer.getMeasuredLatencyData();
	measurer.printMeasurementResults();
	std::string filename(Config::getConfig()->getFilename());
	measurer.saveMeasuredLatencyGnuplotData(filename + "-measured.log");
	measurer.saveReportedLatencyGnuplotData(filename + "-reported.log");
	measurer.saveDiffGnuplotData(filename + "-diff.log");
	setupSucceeded = true;
}

void RosOneShotTimerTests::SetUp()
{
	ASSERT_TRUE(setupSucceeded);
}

