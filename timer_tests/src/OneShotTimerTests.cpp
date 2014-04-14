#include "RosOneShotTimerTestsFixture.h"
#include "main.h"

TEST_F(RosOneShotTimerTests, MaxLatencySmallerThan400MicroSecond)
{
	ASSERT_LE(maxLatencyMs, 400);
}

TEST_F(RosOneShotTimerTests, AverageLatencySmallerThan400MicroSecond)
{
	ASSERT_LE(avgLatencyMs, 400);
}

TEST_F(RosOneShotTimerTests, NoNegativeLatencies)
{
	ASSERT_GE(minLatencyMs, 0);
}

