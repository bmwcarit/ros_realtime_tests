/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "ros/ros.h"
#include "IdleOneShotLatencyMeasurer.h"
#include <time.h>
#include <pthread.h>

#define SEC_TO_NANOSEC_MULTIPLIER 1000000000

void* doRosSpin(void*)
{
	ros::spin();
}

IdleOneShotLatencyMeasurer::IdleOneShotLatencyMeasurer() : OneShotLatencyMeasurer()
{
	pthread_t thread;
	pthread_create(&thread, NULL, &doRosSpin, NULL);
}

bool IdleOneShotLatencyMeasurer::blockUntilCallbackCalled()
{
	callbackCalled = false;
	struct timespec sleep;
	long long timeoutTemp = timeoutNanoseconds * 1.2;
	sleep.tv_sec = (int) timeoutTemp/SEC_TO_NANOSEC_MULTIPLIER;
	sleep.tv_nsec = timeoutTemp - (sleep.tv_sec * SEC_TO_NANOSEC_MULTIPLIER);
	int i = 0;
	while(!callbackCalled)
	{
		nanosleep((const struct timespec*) &sleep, NULL);
		i++;
		if(i>300)
		{
			return false;
		}
	}
	return true;
}

IdleOneShotLatencyMeasurer::~IdleOneShotLatencyMeasurer()
{
}
