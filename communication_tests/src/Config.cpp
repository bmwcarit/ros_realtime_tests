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

Config::Config() : nodeHandle(0), rtPrio(false), fifoScheduling(false), pubFrequency(0), amountMessages(0), namePrefix("")
{
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

std::string Config::getFilename()
{
	std::stringstream filename;
	filename << namePrefix;
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
