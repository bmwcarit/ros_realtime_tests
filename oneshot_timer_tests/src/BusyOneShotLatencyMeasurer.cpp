/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "BusyOneShotLatencyMeasurer.h"

BusyOneShotLatencyMeasurer::BusyOneShotLatencyMeasurer() : OneShotLatencyMeasurer()
{
}

bool BusyOneShotLatencyMeasurer::blockUntilCallbackCalled()
{
	callbackCalled = false;
	while(!callbackCalled && ros::ok())
	{
		ros::spinOnce();
	}
	return true;
}

BusyOneShotLatencyMeasurer::~BusyOneShotLatencyMeasurer()
{
}
