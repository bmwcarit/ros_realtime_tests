/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef PUBSUBLATENCYTESTSFIXTURE_H_
#define PUBSUBLATENCYTESTSFIXTURE_H_

#include "Subscriber.h"
#include <gtest/gtest.h>
#include <rt_tests_support/MeasurementDataEvaluator.h>

class PubSubLatencyTests : public ::testing::Test {
protected:
	static Subscriber* subscriber;
	static MeasurementDataEvaluator* measuredData;

	static void SetUpTestCase();
	static void TearDownTestCase();
};

#endif //PUBSUBLATENCYTESTSFIXTURE
