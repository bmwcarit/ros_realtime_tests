/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Config.h"
#include "ros/ros.h"
#include <sstream>
#include <sys/utsname.h>
#include <rt_tests_support/Logger.h>

Config* Config::configInstance = 0;

Config::Config() : nodeHandle(0), rtPrio(false), fifoScheduling(false), pubFrequency(0), amountMessages(0)
{
}

void Config::printUsage()
{
	Logger::ERROR("Usage: communication_tests_publisher <amount_messages> <pub_frequency(Hz)> [<'rtPrio' + 'FIFO'/'RR'>]");
}

std::string Config::getTitle()
{
	struct utsname unameResponse;
	int rc = uname(&unameResponse);
	std::stringstream machineName;
	if(rc == 0)
	{
		machineName << unameResponse.nodename << " " << unameResponse.sysname << " " << unameResponse.release;
	}
	std::stringstream ss;
	ss << "communication_tests plot " << machineName.str() << " -  " << amountMessages << " samples  ";
	if(rtPrio)
	{
		ss << "test node RT ";
		if(fifoScheduling)
		{
			ss << "FIFO";
		} else {
			ss << "RR";
		}
	}
	return ss.str();
}

bool Config::parseArgs(int argc, char* argv[])
{
	if(argc != 3 && argc != 5)
	{
		return false;
	}
	pubFrequency = atoi(argv[2]);
	amountMessages = atoi(argv[1]);
	if(argc == 5)
	{
		std::string rtPrioString(argv[3]);
		if(strcasecmp(rtPrioString.c_str(), "rtPrio") != 0)
		{
			return false;
		}
		rtPrio = true;
		std::string schedPolicy(argv[4]);
		if(strcasecmp(schedPolicy.c_str(), "RR") == 0)
		{
			fifoScheduling = false;
		} else if(strcasecmp(schedPolicy.c_str(), "FIFO") == 0) 
		{
			Config::getConfig()->fifoScheduling = true;
		} else {
			return false;
		}

	}
	return true;
}

std::string Config::getFilename()
{
	std::stringstream filename;
	filename << "ct_gnuplot_l" << amountMessages << "_fq" << pubFrequency;
	if(rtPrio)
	{
		filename << "-tnRT";
		if(fifoScheduling)
		{
			filename << "FIFO";
		} else {
			filename << "RR";
		}
	}
	return filename.str();
}

Config* Config::getConfig()
{
	if(configInstance == 0)
	{
		configInstance = new Config();
	}
	return configInstance;
}
