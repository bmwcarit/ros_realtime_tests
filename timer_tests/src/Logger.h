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
