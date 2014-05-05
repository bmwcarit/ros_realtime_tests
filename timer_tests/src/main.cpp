/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Config.h"
#include "ros/ros.h"
#include "OneShotLatencyMeasurer.h"
#include <rt_tests_support/Logger.h>

void printUsage()
{
	Logger::ERROR("Usage: timer_tests <amount_measurements> <timeout_value_us> <testnode_rt 0/1> [rt_sched_policy 'FIFO'/'RR']");
}

bool setSchedulingPolicy(std::string arg)
{
	if(arg.length() == 2 && strcasecmp(arg.c_str(), "RR") == 0)
	{
		Config::getConfig()->fifoScheduling = false;
		return true;
	}
	if(arg.length() == 4 && strcasecmp(arg.c_str(), "FIFO") == 0)
	{
		Config::getConfig()->fifoScheduling = true;
		return true;
	}
	return false;
}

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

bool parseArgs(int argc, char* argv[])
{
	if(argc < 4)
	{
		return false;
	}
	Config::getConfig()->loops = atoi(argv[1]);
	Config::getConfig()->timeout_us = atoi(argv[2]);
	Config::getConfig()->testnodeRT = argv[3][0] == '1';
	if(!(argv[3][0] == '0' || argv[3][0] == '1'))
	{
		return false;
	}
	if(Config::getConfig()->testnodeRT)
	{
		if(argc != 5)
		{
			Logger::ERROR("RT Scheduling policy not provided.");
			return false;
		}
		if(!setSchedulingPolicy(argv[4]))
		{
			return false;
		}
	}
	return true;
}

int main(int argc, char* argv[])
{	
	if(!parseArgs(argc, argv))
	{
		printUsage();
		return 1;
	}
	Config* config = Config::getConfig();
	config->testnodePrioritySwitcher = new PrioritySwitcher(0, config->fifoScheduling);
	int x = 1;
	char* y[1];
	y[0] = argv[0];
	if(!setProcessPriority())
	{
		Logger::ERROR("Couldn't set priority appropriately, maybe not running as root?");
		return 1;
	}
	ros::init(x, y, "Timer_tests");
	config->nodeHandle = new ros::NodeHandle;
	Logger::INFO("Performing ROS Timer latency measurements...");
	OneShotLatencyMeasurer measurer(config->loops, config->timeout_us*1000, config->nodeHandle, config->testnodeRT);
	measurer.measure();
	measurer.printMeasurementResults();
	measurer.saveMeasuredLatencyGnuplotData(config->getFilename());
	return 0;
}

