/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef ROSONESHOTTIMERTESTSFIXTURE_H_
#define ROSONESHOTTIMERTESTSFIXTURE_H_

#include <gtest/gtest.h>
#include <rt_tests_support/MeasurementDataEvaluator.h>

class RosOneShotTimerTests : public ::testing::Test {
protected:
	static MeasurementDataEvaluator* measurementData;
	static bool setupSucceeded;

	static void SetUpTestCase();

	virtual void SetUp();
};

#endif //ROSONESHOTTIMERTESTSFIXTURE_H_

