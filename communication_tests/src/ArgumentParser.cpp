/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Config.h"
#include "ArgumentParser.h"
#include <rt_tests_support/Logger.h>

ArgumentParser::ArgumentParser()
{
}

bool ArgumentParser::parseArgs(int argc, char* argv[])
{
	bool initRT = false;
	bool initMsgs = false;
	bool initFreq = false;
	Config* config = Config::getConfig();
	for(int i = 1; i < argc; i++)
	{
		std::string arg(argv[i]);
		if(arg.compare("--rtSched") == 0)
		{
			std::string val(argv[i+1]);
			if(val.compare("0") == 0)
			{
				config->rtPrio = false;
				initRT = true;
			} else if(val.compare("RR") == 0)
			{
				config->rtPrio = true;
				config->fifoScheduling = false;
				initRT = true;
			} else if(val.compare("FIFO") == 0)
			{
				config->rtPrio = true;
				config->fifoScheduling = true;
				initRT = true;
			}
			i++;
		} else if(arg.compare("--messages") == 0)
		{
			config->amountMessages = atoi(argv[i+1]);
			initMsgs = true;
			i++;
		} else if(arg.compare("--frequency") == 0)
		{
			config->pubFrequency = atoi(argv[i+1]);
			initFreq = true;
			i++;
		} else if(arg.compare("--filePrefix") == 0)
		{
			config->namePrefix = std::string(argv[i+1]);
			i++;
		}
	}
	return initRT && initMsgs && initFreq;
}

void ArgumentParser::printUsage()
{
	Logger::ERROR("Args: --frequency <frequency(Hz)> -- messages <amount_messages> --rtSched <0(normalePriority)/RR/FIFO>");
}

ArgumentParser::~ArgumentParser()
{
}
