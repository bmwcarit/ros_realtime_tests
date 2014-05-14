/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef IDLEONESHOTLATENCYMEASURER_H_
#define IDLEONESHOTLATENCYMEASURER_H_

#include "OneShotLatencyMeasurer.h"

class IdleOneShotLatencyMeasurer : public OneShotLatencyMeasurer {
public:
	IdleOneShotLatencyMeasurer();
	~IdleOneShotLatencyMeasurer();
protected:
	void blockUntilCallbackCalled();
};

#endif //IDLEONESHOTLATENCYMEASURER_H_
