/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Config.h"
#include <sstream>

Config* Config::configInstance = 0;

Config::Config() :
	nodeHandle(0),
	testnodePrioritySwitcher(0),
	loops(0),
	timeout_us(0),
	testnodeRT(false),
	fifoScheduling(false)
{
}

std::string Config::getFilename()
{
	std::stringstream filename;
	filename << "Gnuplot_l" << loops << "_Tm" << (int) (timeout_us);
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
