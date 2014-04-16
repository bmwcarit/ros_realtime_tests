/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef LOGGER_H_
#define LOGGER_H_

#include <string>

class Logger {
public:
	static void INFO(std::string msg);
	static void WARN(std::string msg);
	static void ERROR(std::string msg);
};

#endif //LOGGER_H_
