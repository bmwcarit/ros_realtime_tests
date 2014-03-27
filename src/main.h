#ifndef MAIN_H_
#define MAIN_H_

#define S_TO_NS_MULTIPLIER 1000000000

#include "ros/ros.h"
#include "PrioritySwitcher.h"

extern ros::NodeHandle* nodeHandle;
extern PrioritySwitcher* testnodePrioritySwitcher;
extern PrioritySwitcher* roscorePrioritySwitcher;
extern int loops;

#endif //MAIN_H_

