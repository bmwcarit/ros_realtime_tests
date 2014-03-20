#include "RosOneShotTimerTestsFixture.h"
#include "Logger.h"
#include "OneShotLatencyMeasurer.h"
#include "TestParams.h"
#include "main.h"

#include <math.h>

int RosOneShotTimerTestsFixture::minLatencyMs[];
int RosOneShotTimerTestsFixture::maxLatencyMs[];
int RosOneShotTimerTestsFixture::avgLatencyMs[];
bool RosOneShotTimerTestsFixture::setupSucceeded;

void RosOneShotTimerTestsFixture::SetUpTestCase()
{
	Logger::INFO("Performing ROS Timer latency measurements...");
	for(int i = 0; i < amountTimeouts; i++)
	{
		int tmMultiplier = pow(10, i);
		OneShotLatencyMeasurer measurer(loops, 0.0001*tmMultiplier, nodeHandle);
		measurer.measure();
		minLatencyMs[i] = measurer.getMinLatencyMs();
		maxLatencyMs[i] = measurer.getMaxLatencyMs();
		avgLatencyMs[i] = measurer.getAvgLatencyMs();
		measurer.printMeasurementResults();
	}
}

void RosOneShotTimerTestsFixture::SetUp()
{
	ASSERT_TRUE(setupSucceeded);
}

