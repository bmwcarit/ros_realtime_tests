/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "OneShotLatencyMeasurer.h"

#include <stdlib.h>
#include <sys/mman.h>
#include <rt_tests_support/Logger.h>
#include <rt_tests_support/PlotDataFileCreator.h>

#define NANO_TO_MICRO_DIVISOR 1000
#define SEC_TO_NANOSEC_MULTIPLIER 1000000000

OneShotLatencyMeasurer::OneShotLatencyMeasurer(const int loopLength, long timeoutNanoSeconds, ros::NodeHandle* nodeHandle, bool lockMemory) :
		loopLength(loopLength),
		timeoutSeconds(((double)timeoutNanoSeconds)/SEC_TO_NANOSEC_MULTIPLIER),
		timeoutNanoseconds(timeoutSeconds * SEC_TO_NANOSEC_MULTIPLIER),
		nodeHandle(nodeHandle),
		latenciesUs((long*) malloc(sizeof(long)*loopLength)),
		latenciesReportedUs((long*) malloc(sizeof(long)*loopLength)),
		differenceUs((long*) malloc(sizeof(long)*loopLength)),
		callbackCalled(false),
		callbackTs(),
		loopCounter(0),
		lockMemory(lockMemory),
		latencyData(0), reportedLatencyData(0), differenceData(0)
{
	for(int i = 0; i < loopLength; i++)
	{
		latenciesUs[i] = 0;
		latenciesReportedUs[i] = 0;
		differenceUs[i] = 0;
	}
}

void OneShotLatencyMeasurer::measure()
{
	measureOneshotTimerLatencies();
	latencyData = new MeasurementDataEvaluator(latenciesUs, loopLength);
	reportedLatencyData = new MeasurementDataEvaluator(latenciesReportedUs, loopLength);
	differenceData = new MeasurementDataEvaluator(differenceUs, loopLength);
}

void OneShotLatencyMeasurer::measureOneshotTimerLatencies()
{
	if(lockMemory)
	{
		if(mlockall(MCL_CURRENT|MCL_FUTURE) != 0)
		{
			Logger::ERROR("Could'nt lock memory! Aborting...");
			exit(1);
		}
	}
	loopCounter = 0;
	ros::Timer rosTimer = nodeHandle->createTimer(ros::Duration(0.01), &OneShotLatencyMeasurer::timerCallback, this, true);
	spinUntilCallbackCalled();
	long latencyTempNs = 0;
	struct timespec startTs;

	for(loopCounter = 0; loopCounter < loopLength; loopCounter++)
	{
		rosTimer.setPeriod(ros::Duration(timeoutSeconds));
		clock_gettime(clock_id, &startTs);
		rosTimer.start();
		spinUntilCallbackCalled();
		latencyTempNs = ((callbackTs.tv_sec - startTs.tv_sec) * SEC_TO_NANOSEC_MULTIPLIER) + (callbackTs.tv_nsec - startTs.tv_nsec);
		latencyTempNs -= timeoutNanoseconds;
		latenciesUs[loopCounter] = latencyTempNs/NANO_TO_MICRO_DIVISOR;
		differenceUs[loopCounter] = latenciesReportedUs[loopCounter] - latenciesUs[loopCounter];
	}
	if(lockMemory)
	{
		munlockall();
	}
}

MeasurementDataEvaluator* OneShotLatencyMeasurer::getMeasuredLatencyData()
{
	return latencyData;
}

MeasurementDataEvaluator* OneShotLatencyMeasurer::getReportedLatencyData()
{
	return reportedLatencyData;
}

MeasurementDataEvaluator* OneShotLatencyMeasurer::getLatencyDifferenceData()
{
	return differenceData;
}

void OneShotLatencyMeasurer::printMeasurementResults()
{
	std::stringstream ss;
	ss << "Measurement results with a loop length of " << loopLength << " and a timeout of " << (int) (timeoutNanoseconds/1000) << " us:";
	Logger::INFO(ss.str().c_str());
	ss.str("");
	ss <<"Measured:\tMIN:  " << latencyData->getMinValue() << "us \tAVG:  " << latencyData->getAvgValue() << "us \tMAX:  " << latencyData->getMaxValue() << "us";
	Logger::INFO(ss.str().c_str());
	ss.str("");
	ss <<"Reported:\tMIN:  " << reportedLatencyData->getMinValue() << "us \tAVG:  " << reportedLatencyData->getAvgValue() << "us \tMAX:  " << reportedLatencyData->getMaxValue() << "us";
	Logger::INFO(ss.str().c_str());
	ss.str("");
	ss << "Difference:\tMIN: " << differenceData->getMinValue() << "us\tAVG: " << differenceData->getAvgValue() << "us\tMAX: " << differenceData->getMaxValue() << "us";
	Logger::INFO(ss.str().c_str());
}

void OneShotLatencyMeasurer::saveMeasuredLatencyGnuplotData(std::string filename)
{
	saveGnuplotData(filename, latencyData);
}

void OneShotLatencyMeasurer::saveReportedLatencyGnuplotData(std::string filename)
{
	saveGnuplotData(filename, reportedLatencyData);
}
void OneShotLatencyMeasurer::saveDiffGnuplotData(std::string filename)
{
	saveGnuplotData(filename, differenceData);
}

void OneShotLatencyMeasurer::saveGnuplotData(std::string filename, MeasurementDataEvaluator* plotData)
{
	std::stringstream ss;
	ss << "# Plot data for gnuplot" << std::endl;
	ss << "# Timeout: " << (int) timeoutNanoseconds/1000 << "us, LoopLength: " << loopLength << std::endl;
	ss << "# Measured:\t MIN: " << latencyData->getMinValue() << "us \tAVG:  " << latencyData->getAvgValue() << "us \tMAX:  " << latencyData->getMaxValue() << "us" << std::endl;
	ss << "# Reported:\t MIN: " << reportedLatencyData->getMinValue() << "us \tAVG:  " << reportedLatencyData->getAvgValue() << "us \tMAX:  " << reportedLatencyData->getMaxValue() << "us" << std::endl;
	ss << "# Difference:\t MIN: " << differenceData->getMinValue() << "us\tAVG: " << differenceData->getAvgValue() << "us\tMAX: " << differenceData->getMaxValue() << "us" << std::endl;
	PlotDataFileCreator plotter;
	plotter.createPlottableDatafile(filename, ss.str(), plotData);
}

void OneShotLatencyMeasurer::timerCallback(const ros::TimerEvent& te)
{
	clock_gettime(clock_id, &callbackTs);
	long latencyReportedTemp = ((te.current_real.sec - te.current_expected.sec) * SEC_TO_NANOSEC_MULTIPLIER) + (te.current_real.nsec - te.current_expected.nsec);
	latenciesReportedUs[loopCounter] = latencyReportedTemp/NANO_TO_MICRO_DIVISOR;
	callbackCalled = true;
}

void OneShotLatencyMeasurer::spinUntilCallbackCalled()
{
	callbackCalled = false;
	while(!callbackCalled && ros::ok())
	{
		ros::spinOnce();
	}
}

OneShotLatencyMeasurer::~OneShotLatencyMeasurer()
{
	delete differenceUs;
	delete latenciesReportedUs;
	delete latenciesUs;
	delete latencyData, reportedLatencyData, differenceData;
}

