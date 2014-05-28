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
#include "ArgumentParser.h"
#include <rt_tests_support/Logger.h>
#include <rt_tests_support/PrioritySwitcher.h>

int main(int argc, char* argv[])
{
	Config* config = Config::getConfig();
	ArgumentParser argParser;
	if(!argParser.parseArgs(argc, argv))
	{
		argParser.printUsage();
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
	ros::init(argc, argv, "communication_tests_publisher");
	config->nodeHandle = new ros::NodeHandle();
	Publisher publisher("communication_tests");
	sleep(config->startDelay);
	publisher.publish();
	ros::shutdown();
	Logger::INFO("Done publishing...");
	return 0;
}
