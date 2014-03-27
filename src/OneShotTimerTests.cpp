#include "RosOneShotTimerTestsFixture.h"
#include "main.h"

class RosOneShotTimerTests : public RosOneShotTimerTestsFixture {
protected:

	static void SetUpTestCase()
	{
		setupSucceeded = false;
		ASSERT_EQ(0, testnodePrioritySwitcher->switchToNormalPriority());
		if(roscorePrioritySwitcher != NULL)
		{
			ASSERT_EQ(0, roscorePrioritySwitcher->switchToNormalPriority());
		}
		rtState = false;
		RosOneShotTimerTestsFixture::SetUpTestCase();
		setupSucceeded = true;
	}
};

TEST_F(RosOneShotTimerTests, MaxLatencySmallerThan400MicroSecond)
{
	for(int i = 0; i < amountTimeouts; i++)
	{
		ASSERT_LE(maxLatencyMs[i], 400);
	}
}

TEST_F(RosOneShotTimerTests, AverageLatencySmallerThan400MicroSecond)
{
	for(int i = 0; i < amountTimeouts; i++)
	{
		ASSERT_LE(avgLatencyMs[i], 400);
	}
}

TEST_F(RosOneShotTimerTests, NoNegativeLatencies)
{
	for(int i = 0; i < amountTimeouts; i++)
	{
		ASSERT_GE(minLatencyMs[i], 0);
	}
}

