/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Config.h"
#include <sstream>
#include <sys/utsname.h>

Config* Config::configInstance = 0;

Config::Config() :
	nodeHandle(0),
	testnodePrioritySwitcher(0),
	loops(0),
	frequency(0),
	testnodeRT(false),
	fifoScheduling(false),
	namePrefix("")
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
	ss << "cyclic_timer_tests plot " << machineName.str() << " -  " << loops << " samples;  Frequency: ";
	ss << frequency << "Hz";
	if(testnodeRT)
	{
		ss << ";  test node RT ";
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
	filename << "cyclic_timer_tests_l" << loops << "_fq" << frequency;
	if(testnodeRT)
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
