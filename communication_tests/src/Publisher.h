/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#include "ros/ros.h"
#include <string>

class Publisher {
public:
	Publisher(const std::string& topic, ros::NodeHandle* nodeHandle);
	void publish(int frequency, int amount);
	~Publisher();
private:
	Publisher();
	ros::NodeHandle* nodeHandle;
	ros::Publisher rosPublisher;
};

#endif //PUBLISHER_H_
