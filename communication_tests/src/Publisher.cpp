/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Config.h"
#include "Publisher.h"
#include <rt_tests_support/Logger.h>
#include <communication_tests/timestamp_msg.h>

Publisher::Publisher(const std::string& topic) :
	nodeHandle(Config::getConfig()->nodeHandle),
	rosPublisher(nodeHandle->advertise<communication_tests::timestamp_msg>(topic, 1000))
{
}

void Publisher::publish()
{
	Config* config = Config::getConfig();
	ros::Rate publishFrequency(config->pubFrequency);
	int sequenceNumber = 0;
	communication_tests::timestamp_msg message;
	message.payload.clear();
	for(int i = 0; i < config->payloadLength; i++)
	{
		message.payload.push_back(0xFF);
	}
	for(int i = 0; i < config->amountMessages; i++)
	{
		message.seq = i;
		message.last_msg = (i == (config->amountMessages-1));
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
