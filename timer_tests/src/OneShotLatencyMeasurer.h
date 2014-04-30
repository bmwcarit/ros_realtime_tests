/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef ONESHOTLATENCYMEASURER_H_
#define ONESHOTLATENCYMEASURER_H_

#include "ros/ros.h"
#include <string>
#include <time.h>
#include <rt_tests_support/MeasurementDataEvaluator.h>

class OneShotLatencyMeasurer {
public:
	OneShotLatencyMeasurer(const int loopLength, long timeoutNanoSeconds, ros::NodeHandle* nodeHandle, bool lockMemory);
	~OneShotLatencyMeasurer();
	void measure();
	void printMeasurementResults();
	void saveDiffGnuplotData(std::string filename);
	void saveMeasuredLatencyGnuplotData(std::string filename);
	void saveReportedLatencyGnuplotData(std::string filename);
	const static int clock_id = CLOCK_MONOTONIC_RAW;

	MeasurementDataEvaluator* getMeasuredLatencyData();
	MeasurementDataEvaluator* getReportedLatencyData();
	MeasurementDataEvaluator* getLatencyDifferenceData();
private:
	const int loopLength;
	const double timeoutSeconds;
	const long timeoutNanoseconds;
	ros::NodeHandle* nodeHandle;
	bool callbackCalled;
	struct timespec callbackTs;
	int loopCounter;
	const bool lockMemory;
	MeasurementDataEvaluator* latencyData;
	MeasurementDataEvaluator* reportedLatencyData;
	MeasurementDataEvaluator* differenceData; //reported - measured

	void saveGnuplotData(std::string filename, MeasurementDataEvaluator* plotData);
	void measureOneshotTimerLatencies();
	void timerCallback(const ros::TimerEvent&);
	void spinUntilCallbackCalled();
};

#endif //ONESHOTLATENCYMEASURER_H_

