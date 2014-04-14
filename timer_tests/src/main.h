/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef MAIN_H_
#define MAIN_H_

#define S_TO_NS_MULTIPLIER 1000000000

#include "ros/ros.h"
#include "PrioritySwitcher.h"

extern ros::NodeHandle* nodeHandle;
extern PrioritySwitcher* testnodePrioritySwitcher;
extern int loops;
extern int timeout_us;
extern bool testnodeRT;
extern bool fifoScheduling;

#endif //MAIN_H_

