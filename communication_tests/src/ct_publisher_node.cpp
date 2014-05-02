/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Config.h"
#include "ros/ros.h"
#include "Publisher.h"
#include <rt_tests_support/Logger.h>

void printUsage()
{
	Logger::ERROR("Usage: communication_tests_publisher <amount_messages> <pub_frequency(Hz)>");
}

int main(int argc, char* argv[])
{
	if(argc != 3)
	{
		printUsage();
		return 1;
	}
	int x = 1;
	char* y[1];
	y[0] = argv[0];
	ros::init(x, y, "communication_tests_publisher");
	Config* config = Config::getConfig();
	config->nodeHandle = new ros::NodeHandle();
	config->pubFrequency = atoi(argv[2]);
	config->amountMessages = atoi(argv[1]);
	Publisher publisher("communication_tests", config->nodeHandle);
	sleep(2);
	publisher.publish(config->pubFrequency, config->amountMessages);
	Logger::INFO("Done publishing...");
	return 0;
}
