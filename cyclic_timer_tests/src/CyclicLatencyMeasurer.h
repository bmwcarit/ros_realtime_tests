/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef CYCLICLATENCYMEASURER_H_
#define CYCLICLATENCYMEASURER_H_

#include "Config.h"
#include "ros/ros.h"
#include <string>
#include <time.h>
#include <rt_tests_support/MeasurementDataEvaluator.h>

class CyclicLatencyMeasurer {
public:
	CyclicLatencyMeasurer();
	~CyclicLatencyMeasurer();
	void measure();
	void printMeasurementResults();
	void savePlottableData();
	const static int clock_id = CLOCK_MONOTONIC_RAW;
private:
	Config* config;
	int callbackCounter;
	MeasurementDataEvaluator* jitter;
	struct timespec* callbackTimestamps;

	void measureTimerJitter();
	void timerCallback(const ros::TimerEvent&);
	std::string getMeasurementSummary();
};

#endif //CYCLICLATENCYMEASURER_H_
