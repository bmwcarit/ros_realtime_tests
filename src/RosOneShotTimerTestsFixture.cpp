#include "RosOneShotTimerTestsFixture.h"
#include "Logger.h"
#include "OneShotLatencyMeasurer.h"
#include "TestParams.h"
#include "main.h"

#include <math.h>

int RosOneShotTimerTests::minLatencyMs[];
int RosOneShotTimerTests::maxLatencyMs[];
int RosOneShotTimerTests::avgLatencyMs[];
bool RosOneShotTimerTests::setupSucceeded;

void RosOneShotTimerTests::SetUpTestCase()
{
	Logger::INFO("Performing ROS Timer latency measurements...");
	setupSucceeded = false;
	if(testnodeRT)
	{
		ASSERT_EQ(0, testnodePrioritySwitcher->switchToRealtimePriority());
	} else {
		ASSERT_EQ(0, testnodePrioritySwitcher->switchToNormalPriority());
	}
	if(roscoreRT)
	{
		ASSERT_EQ(0, roscorePrioritySwitcher->switchToRealtimePriority());
	} else {
		ASSERT_EQ(0, roscorePrioritySwitcher->switchToNormalPriority());
	}
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
		if(testnodeRT)
		{
			filenameSS << "-tnRT";
		}		
		if(roscoreRT)
		{
			filenameSS << "-rcRT";
		}
		measurer.saveMeasuredLatencyGPlotData(filenameSS.str() + "-measured.log");
		measurer.saveReportedLatencyGPlotData(filenameSS.str() + "-reported.log");
		measurer.saveDiffGPlotData(filenameSS.str() + "-diff.log");
	}
	setupSucceeded = true;
}

void RosOneShotTimerTests::SetUp()
{
	ASSERT_TRUE(setupSucceeded);
}

