/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Config.h"
#include "ros/ros.h"
#include "ArgumentParser.h"
#include "BusyOneShotLatencyMeasurer.h"
#include "IdleOneShotLatencyMeasurer.h"
#include <rt_tests_support/Logger.h>

bool setProcessPriority()
{
	int rc = 0;
	if(Config::getConfig()->testnodeRT)
	{
		rc += Config::getConfig()->testnodePrioritySwitcher->switchToRealtimePriority();
	} else {
		rc += Config::getConfig()->testnodePrioritySwitcher->switchToNormalPriority();
	}
	return rc == 0;
}

int main(int argc, char* argv[])
{
	ArgumentParser argParser;
	if(!argParser.parseArgs(argc, argv))
	{
		Logger::ERROR("Invalid arguments provided!");
		Logger::ERROR(argParser.getUsage());
		return 1;
	}
	Config* config = Config::getConfig();
	config->testnodePrioritySwitcher = new PrioritySwitcher(0, config->fifoScheduling);
	if(!setProcessPriority())
	{
		Logger::ERROR("Couldn't set priority appropriately, maybe not running as root?");
		return 1;
	}
	ros::init(argc, argv, "timer_tests");
	config->nodeHandle = new ros::NodeHandle;
	Logger::INFO("Performing ROS Timer latency measurements...");
	OneShotLatencyMeasurer* measurer;
	if(config->busyMode)
	{
		Logger::INFO("Running in busy mode");
		measurer = new BusyOneShotLatencyMeasurer();
	} else {
		Logger::INFO("Running in default mode");
		measurer = new IdleOneShotLatencyMeasurer();
	}
	measurer->measure();
	measurer->printMeasurementResults();
	measurer->saveMeasuredLatencyGnuplotData();
	return 0;
}

