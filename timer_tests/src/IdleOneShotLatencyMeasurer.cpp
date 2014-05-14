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

void IdleOneShotLatencyMeasurer::blockUntilCallbackCalled()
{
	callbackCalled = false;
	struct timespec sleep;
	sleep.tv_sec = (int) timeoutSeconds;
	sleep.tv_nsec = timeoutNanoseconds - (sleep.tv_sec * SEC_TO_NANOSEC_MULTIPLIER);
	while(!callbackCalled)
	{
		nanosleep((const struct timespec*) &sleep, NULL);
	}
}

IdleOneShotLatencyMeasurer::~IdleOneShotLatencyMeasurer()
{
}
