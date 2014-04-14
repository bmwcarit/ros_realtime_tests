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
