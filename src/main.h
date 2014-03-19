#ifndef MAIN_H_
#define MAIN_H_

#define S_TO_NS_MULTIPLIER 1000000000

#include "ros/ros.h"

#include <gtest/gtest.h>
#include <time.h>
#include <unistd.h>

extern ros::NodeHandle* nodeHandle;
extern struct timespec callbackTs;

void timerCallback(const ros::TimerEvent&);
void spinUntilCallbackCalled();


#endif //MAIN_H_
