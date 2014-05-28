/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef ONESHOTLATENCYMEASURER_H_
#define ONESHOTLATENCYMEASURER_H_

#include "Config.h"
#include "ros/ros.h"
#include <string>
#include <time.h>
#include <rt_tests_support/MeasurementDataEvaluator.h>

class OneShotLatencyMeasurer {
public:
	OneShotLatencyMeasurer();
	~OneShotLatencyMeasurer();
	void measure();
	void printMeasurementResults();
	void saveDiffGnuplotData();
	void saveMeasuredLatencyGnuplotData();
	void saveReportedLatencyGnuplotData();
	const static int clock_id = CLOCK_MONOTONIC_RAW;

	MeasurementDataEvaluator* getMeasuredLatencyData();
	MeasurementDataEvaluator* getReportedLatencyData();
	MeasurementDataEvaluator* getLatencyDifferenceData();
protected:
	const int loopLength;
	const long timeoutNanoseconds;
	const double timeoutSeconds;
	ros::NodeHandle* nodeHandle;
	bool callbackCalled;
	struct timespec callbackTs;
	int loopCounter;
	int ignoredTimerCounter;
	const bool lockMemory;
	MeasurementDataEvaluator* latencyData;
	MeasurementDataEvaluator* reportedLatencyData;
	MeasurementDataEvaluator* differenceData; //reported - measured

	void saveGnuplotData(std::string filename, MeasurementDataEvaluator* plotData);
	void measureOneshotTimerLatencies();
	void timerCallback(const ros::TimerEvent&);
	std::string getMeasurementSummary();
	virtual bool blockUntilCallbackCalled()=0;
};

#endif //ONESHOTLATENCYMEASURER_H_

