/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef SUBSCRIBER_H_
#define SUBSCRIBER_H_

#include "ros/ros.h"
#include <string>
#include <communication_tests/timestamp_msg.h>
#include <rt_tests_support/MeasurementDataEvaluator.h>

class Subscriber{
public:
	Subscriber(const std::string& topic, ros::NodeHandle* nodeHandle, int amountMessages);
	void startMeasurement();
	void printMeasurementResults();
	void saveGnuplotData(std::string filename);
	int getAmountMessagesOutOfOrder();
	MeasurementDataEvaluator* getMeasurementData();
	~Subscriber();
private:
	Subscriber();
	int lastSeq;
	int outOfOrderCounter;
	long* latenciesUs;
	const int amountMessages;
	const static int messageMissing = -1;
	ros::NodeHandle* nodeHandle;
	ros::Subscriber rosSubscriber;
	MeasurementDataEvaluator* measurementData;
	std::string getMeasurementSummary();
	void messageCallback(const communication_tests::timestamp_msg::ConstPtr& msg);
};

#endif //SUBSCRIBER_H_
