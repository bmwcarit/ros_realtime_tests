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
	Config* config = Config::getConfig();
	config->rtPrio = false;
	config->fifoScheduling = false;
	config->amountMessages = 1000;
	config->pubFrequency = 1000;
	config->payloadLength = 0;
	config->startDelay = 0;
	for(int i = 1; i < argc; i++)
	{
		std::string arg(argv[i]);
		if(arg.compare("-s") == 0)
		{
			std::string val(argv[i+1]);
			if(val.compare("0") == 0)
			{
				config->rtPrio = false;
			} else if(val.compare("RR") == 0)
			{
				config->rtPrio = true;
				config->fifoScheduling = false;
			} else if(val.compare("FIFO") == 0)
			{
				config->rtPrio = true;
				config->fifoScheduling = true;
			} else {
				return false;
			}
			i++;
		} else if(arg.compare("-r") == 0)
		{
			config->amountMessages = atoi(argv[i+1]);
			i++;
		} else if(arg.compare("-f") == 0)
		{
			config->pubFrequency = atoi(argv[i+1]);
			i++;
		} else if(arg.compare("-l") == 0)
		{
			config->payloadLength = atoi(argv[i+1]);
			i++;
		} else if(arg.compare("-p") == 0)
		{
			config->namePrefix = std::string(argv[i+1]);
			i++;
		} else if(arg.compare("-d") == 0)
		{
			config->startDelay = atoi(argv[i+1]);
			i++;
		} else if(arg.compare("-t") == 0)
		{
			config->topic = std::string(argv[i+1]);
			i++;
		}
	}
	return true;
}

void ArgumentParser::printUsage()
{
	Logger::ERROR("Args: [-f <frequency>] [-r <repetitions>] [-s <0/RR/FIFO>] [-l <payload_length>] [-p <file_prefix>] [-d <start_delay(seconds)>] [-t <topic>]");
}

ArgumentParser::~ArgumentParser()
{
}
