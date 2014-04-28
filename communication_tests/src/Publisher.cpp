/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Publisher.h"
#include <rt_tests_support/Logger.h>
#include <communication_tests/timestamp_msg.h>

Publisher::Publisher(const std::string& topic, ros::NodeHandle* nodeHandle) :
	nodeHandle(nodeHandle),
	rosPublisher(nodeHandle->advertise<communication_tests::timestamp_msg>(topic, 1000))
{
}

void Publisher::publish(int frequency, int amount)
{
	ros::Rate publishFrequency(frequency);
	int sequenceNumber = 0;
	for(int i = 0; i < amount; i++)
	{
		communication_tests::timestamp_msg message;
		message.seq = i;
		message.last_msg = (i == (amount-1));
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
		message.sec = ts.tv_sec;
		message.nsec = ts.tv_nsec;
		rosPublisher.publish(message);
		publishFrequency.sleep();
	}
}

Publisher::~Publisher()
{
}
