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

	//Getter methods
	int getMaxLatencyUs();
	int getMinLatencyUs();
	int getAvgLatencyUs();
	int getMaxReportedLatencyUs();
	int getMinReportedLatencyUs();
	int getAvgReportedLatencyUs();
	int getMaxDifferenceUs();
	int getMinDifferenceUs();
	int getAvgDifferenceAbsUs();
private:
	const int loopLength;
	const double timeoutSeconds;
	const long timeoutNanoseconds;
	long minLatencyNs;
	long maxLatencyNs;
	unsigned long long avgLatencyNs;
	long minLatencyReportedNs;
	long maxLatencyReportedNs;
	unsigned long long avgLatencyReportedNs;
	ros::NodeHandle* nodeHandle;
	long* latenciesNs;
	long* latenciesReportedNs;
	long* differenceNs;
	bool callbackCalled;
	struct timespec callbackTs;
	int loopCounter;
	long maxDifference;
	long minDifference;
	unsigned long long avgDifferenceAbs;
	const bool lockMemory;

	void saveGnuplotData(std::string filename, long* plotValues, int maxValueMs, int minValueMs);
	void measureOneshotTimerLatencies();
	void calcMinMaxAndAvg();
	void timerCallback(const ros::TimerEvent&);
	void spinUntilCallbackCalled();
};

#endif //ONESHOTLATENCYMEASURER_H_

