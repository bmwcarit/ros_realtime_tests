/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Config.h"
#include "PubSubLatencyTestsFixture.h"
#include <sstream>

Subscriber* PubSubLatencyTests::subscriber;
MeasurementDataEvaluator* PubSubLatencyTests::measuredData;

void PubSubLatencyTests::SetUpTestCase()
{
	subscriber = new Subscriber("communication_tests", new ros::NodeHandle(), Config::getConfig()->amountMessages);
	subscriber->startMeasurement();
	subscriber->printMeasurementResults();
	measuredData = subscriber->getMeasurementData();
	subscriber->saveGnuplotData(Config::getConfig()->getFilename());
}

void PubSubLatencyTests::TearDownTestCase()
{
	delete subscriber;
}
