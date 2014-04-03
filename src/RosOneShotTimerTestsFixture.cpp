#include "RosOneShotTimerTestsFixture.h"
#include "Logger.h"
#include "OneShotLatencyMeasurer.h"
#include "main.h"

#include <math.h>

int RosOneShotTimerTests::minLatencyMs;
int RosOneShotTimerTests::maxLatencyMs;
int RosOneShotTimerTests::avgLatencyMs;
bool RosOneShotTimerTests::setupSucceeded;

void RosOneShotTimerTests::SetUpTestCase()
{
	Logger::INFO("Performing ROS Timer latency measurements...");
	setupSucceeded = false;
	OneShotLatencyMeasurer measurer(loops, timeout_us*1000, nodeHandle, testnodeRT);
	measurer.measure();
	minLatencyMs = measurer.getMinLatencyMs();
	maxLatencyMs = measurer.getMaxLatencyMs();
	avgLatencyMs = measurer.getAvgLatencyMs();
	measurer.printMeasurementResults();
	std::stringstream filenameSS;
	filenameSS << "GPlot_l" << loops << "_Tm" << (int) (timeout_us);
	if(testnodeRT)
	{
		filenameSS << "-tnRT";
		if(fifoScheduling)
		{
			filenameSS << "FIFO";
		} else {
			filenameSS << "RR";
		}
	}
	measurer.saveMeasuredLatencyGPlotData(filenameSS.str() + "-measured.log");
	measurer.saveReportedLatencyGPlotData(filenameSS.str() + "-reported.log");
	measurer.saveDiffGPlotData(filenameSS.str() + "-diff.log");
	setupSucceeded = true;
}

void RosOneShotTimerTests::SetUp()
{
	ASSERT_TRUE(setupSucceeded);
}

