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
bool RosOneShotTimerTestsFixture::rtState;

void RosOneShotTimerTestsFixture::SetUpTestCase()
{
	Logger::INFO("Performing ROS Timer latency measurements...");
	for(int i = 0; i < amountTimeouts; i++)
	{
		int tmMultiplier = pow(10, i);
		const long timeout = 100000*tmMultiplier;
		OneShotLatencyMeasurer measurer(loops, timeout, nodeHandle);
		measurer.measure();
		minLatencyMs[i] = measurer.getMinLatencyMs();
		maxLatencyMs[i] = measurer.getMaxLatencyMs();
		avgLatencyMs[i] = measurer.getAvgLatencyMs();
		measurer.printMeasurementResults();
		std::stringstream filenameSS;
		filenameSS << "GPlot_l" << loops << "_Tm" << (int) (timeout/1000);
		if(rtState)
		{
			filenameSS << "-RT";
		}
		filenameSS << ".log";
		measurer.saveGPlotData(filenameSS.str());
	}
}

void RosOneShotTimerTestsFixture::SetUp()
{
	ASSERT_TRUE(setupSucceeded);
}

