/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "PubSubLatencyTestsFixture.h"

TEST_F(PubSubLatencyTests, NoMissingMessages)
{
	ASSERT_NE(-1, measuredData->getMinValue());
}

TEST_F(PubSubLatencyTests, NoMessagesOutOfOrder)
{
	ASSERT_EQ(0, subscriber->getAmountMessagesOutOfOrder());
}

TEST_F(PubSubLatencyTests, AverageLatencyBelowOneMillisecond)
{
	ASSERT_LT(measuredData->getAvgValue(), 1000000);
}

TEST_F(PubSubLatencyTests, MaximumLatencyBelowTwoMilliseconds)
{
	ASSERT_LT(measuredData->getMaxValue(), 2000000);
}
