/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef BUSYONESHOTLATENCYMEASURER_H_
#define BUSYONESHOTLATENCYMEASURER_H_

#include "OneShotLatencyMeasurer.h"

class BusyOneShotLatencyMeasurer : public OneShotLatencyMeasurer {
public:
	BusyOneShotLatencyMeasurer();
	~BusyOneShotLatencyMeasurer();
protected:
	void blockUntilCallbackCalled();
};

#endif //BUSYONESHOTLATENCYMEASURER_H_
