/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "main.h"
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
	OneShotLatencyMeasurer measurer(loops, timeout_us*1000, nodeHandle, testnodeRT);
	measurer.measure();
	measurementData = measurer.getMeasuredLatencyData();
	measurer.printMeasurementResults();
	std::stringstream filenameSS;
	filenameSS << "Gnuplot_l" << loops << "_Tm" << (int) (timeout_us);
	if(testnodeRT)
	{
		filenameSS << "-tnRT";
		if(fifoScheduling)
		{
			filenameSS << "FIFO";
		} else {
			filenameSS << "RR";
		}
	}
	measurer.saveMeasuredLatencyGnuplotData(filenameSS.str() + "-measured.log");
	measurer.saveReportedLatencyGnuplotData(filenameSS.str() + "-reported.log");
	measurer.saveDiffGnuplotData(filenameSS.str() + "-diff.log");
	setupSucceeded = true;
}

void RosOneShotTimerTests::SetUp()
{
	ASSERT_TRUE(setupSucceeded);
}

