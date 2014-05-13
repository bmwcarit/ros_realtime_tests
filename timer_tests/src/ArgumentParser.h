/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef ARGUMENT_PARSER_H_
#define ARGUMENT_PARSER_H_

#include <string>

class ArgumentParser{
public:
	ArgumentParser();
	bool parseArgs(int argc, char* argv[]);
	std::string getUsage();
	~ArgumentParser();
};

#endif //ARGUMENT_PARSER_H_
