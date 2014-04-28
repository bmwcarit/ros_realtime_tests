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

#define SEC_TO_NANOSEC_MULTIPLIER 1000000000

OneShotLatencyMeasurer::OneShotLatencyMeasurer(const int loopLength, long timeoutNanoSeconds, ros::NodeHandle* nodeHandle, bool lockMemory) :
		loopLength(loopLength),
		timeoutSeconds(((double)timeoutNanoSeconds)/SEC_TO_NANOSEC_MULTIPLIER),
		timeoutNanoseconds(timeoutSeconds * SEC_TO_NANOSEC_MULTIPLIER),
		nodeHandle(nodeHandle),
		latenciesNs((long*) malloc(sizeof(long)*loopLength)),
		latenciesReportedNs((long*) malloc(sizeof(long)*loopLength)),
		differenceNs((long*) malloc(sizeof(long)*loopLength)),
		callbackCalled(false),
		callbackTs(),
		loopCounter(0),
		lockMemory(lockMemory),
		latencyData(0), reportedLatencyData(0), differenceData(0)
{
	for(int i = 0; i < loopLength; i++)
	{
		latenciesNs[i] = 0;
		latenciesReportedNs[i] = 0;
		differenceNs[i] = 0;
	}
}

void OneShotLatencyMeasurer::measure()
{
	measureOneshotTimerLatencies();
	latencyData = new MeasurementDataEvaluator(latenciesNs, loopLength);
	reportedLatencyData = new MeasurementDataEvaluator(latenciesReportedNs, loopLength);
	differenceData = new MeasurementDataEvaluator(differenceNs, loopLength);
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
		latenciesNs[loopCounter] = latencyTempNs;
		differenceNs[loopCounter] = latenciesReportedNs[loopCounter] - latenciesNs[loopCounter];
	}
	if(lockMemory)
	{
		munlockall();
	}
}

void OneShotLatencyMeasurer::printMeasurementResults()
{
	std::stringstream ss;
	ss << "Measurement results with a loop length of " << loopLength << " and a timeout of " << (int) (timeoutNanoseconds/1000) << " us:";
	Logger::INFO(ss.str().c_str());
	ss.str("");
	ss <<"Measured:\tMIN:  " << getMinLatencyUs() << "us \tAVG:  " << getAvgLatencyUs() << "us \tMAX:  " << getMaxLatencyUs() << "us";
	Logger::INFO(ss.str().c_str());
	ss.str("");
	ss <<"Reported:\tMIN:  " << getMinReportedLatencyUs() << "us \tAVG:  " << getAvgReportedLatencyUs() << "us \tMAX:  " << getMaxReportedLatencyUs() << "us";
	Logger::INFO(ss.str().c_str());
	ss.str("");
	ss << "Difference:\tMIN: " << getMinDifferenceUs() << "us\tAVG: " << getAvgDifferenceAbsUs() << "us\tMAX: " << getMaxDifferenceUs() << "us";
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
	ss << "# Measured:\t MIN: " << getMinLatencyUs()  << "us \tAVG: " << getAvgLatencyUs() << "us \tMAX: " << getMaxLatencyUs() << "us" << std::endl;
	ss << "# Reported:\t MIN: " << getMinReportedLatencyUs()  << "us \tAVG: " << getAvgReportedLatencyUs() << "us \tMAX: " << getMaxReportedLatencyUs() << "us" << std::endl;
	ss << "# Difference:\t MIN: " << getMinDifferenceUs()  << "us \tAVG: " << getAvgDifferenceAbsUs() << "us \tMAX: " << getMaxDifferenceUs() << "us" << std::endl;
	PlotDataFileCreator plotter;
	plotter.createPlottableDatafile(filename, ss.str(), plotData, 1000);
}

int OneShotLatencyMeasurer::getMaxLatencyUs()
{
	return latencyData->getMaxValue()/1000;
}

int OneShotLatencyMeasurer::getMinLatencyUs()
{
	return latencyData->getMinValue()/1000;
}

int OneShotLatencyMeasurer::getAvgLatencyUs()
{
	return latencyData->getAvgValue()/1000;
}

int OneShotLatencyMeasurer::getMaxReportedLatencyUs()
{
	return reportedLatencyData->getMaxValue()/1000;
}

int OneShotLatencyMeasurer::getMinReportedLatencyUs()
{
	return reportedLatencyData->getMinValue()/1000;
}

int OneShotLatencyMeasurer::getAvgReportedLatencyUs()
{
	return reportedLatencyData->getAvgValue()/1000;
}

int OneShotLatencyMeasurer::getMaxDifferenceUs()
{
	return differenceData->getMaxValue()/1000;
}

int OneShotLatencyMeasurer::getMinDifferenceUs()
{
	return differenceData->getMinValue()/1000;
}

int OneShotLatencyMeasurer::getAvgDifferenceAbsUs()
{
	return differenceData->getAvgValue()/1000;
}

void OneShotLatencyMeasurer::timerCallback(const ros::TimerEvent& te)
{
	clock_gettime(clock_id, &callbackTs);
	latenciesReportedNs[loopCounter] = ((te.current_real.sec - te.current_expected.sec) * SEC_TO_NANOSEC_MULTIPLIER) + (te.current_real.nsec - te.current_expected.nsec);
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
	delete differenceNs;
	delete latenciesReportedNs;
	delete latenciesNs;
	delete latencyData, reportedLatencyData, differenceData;
}

