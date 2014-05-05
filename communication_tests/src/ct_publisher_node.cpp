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
#include <rt_tests_support/PrioritySwitcher.h>

int main(int argc, char* argv[])
{
	Config* config = Config::getConfig();
	if(!config->parseArgs(argc, argv))
	{
		config->printUsage();
		return 1;
	}
	if(config->rtPrio)
	{
		PrioritySwitcher prioSwitcher(config->fifoScheduling);
		if(prioSwitcher.switchToRealtimePriority() != 0)
		{
			Logger::ERROR("Switching to realtime priority failed, maybe not running as root?");
			return 1;
		}
	}
	int x = 1;
	char* y[1];
	y[0] = argv[0];
	ros::init(x, y, "communication_tests_publisher");
	config->nodeHandle = new ros::NodeHandle();
	Publisher publisher("communication_tests", config->nodeHandle);
	sleep(2);
	publisher.publish(config->pubFrequency, config->amountMessages);
	Logger::INFO("Done publishing...");
	return 0;
}
