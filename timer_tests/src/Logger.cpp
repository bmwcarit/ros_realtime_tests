/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Logger.h"

#include <iostream>

void Logger::INFO(std::string msg)
{
	std::cout << "\tINFO: " << msg << std::endl;
}

void Logger::WARN(std::string msg)
{
	std::cout << "\tWARN: " << msg << std::endl;
}

void Logger::ERROR(std::string msg)
{
	std::cerr << "\tERROR: " << msg << std::endl;
}
