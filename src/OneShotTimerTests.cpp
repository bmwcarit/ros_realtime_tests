#include "main.h"
#include "Logger.h"
#include "TestParams.h"


void measureOneshotTimerLatencies();
void calcMaxAndAvgLatency();
long latenciesNs[OSTIMER_TEST_RUNS];
long minLatencyMs;
long maxLatencyMs;
long avgLatencyMs;

class RosOneShotTimerTest : public ::testing::Test {
protected:
	static void SetUpTestCase()
	{
		Logger::INFO("Running timer latency measurements...");
		measureOneshotTimerLatencies();
		Logger::INFO("Calculating max and avg...");
		calcMaxAndAvgLatency();
		Logger::INFO("OSTimer Setup Done.");
		std::stringstream ss;
		ss << "Min measured latency: " << minLatencyMs << " microseconds.";
		Logger::INFO(ss.str().c_str());
		ss.str("");
		ss << "Max measured latency: " << maxLatencyMs << " microseconds.";
		Logger::INFO(ss.str().c_str());
		ss.str("");
		ss << "Avg measured latency: " << avgLatencyMs << " microseconds.";
		Logger::INFO(ss.str().c_str());
	}
};

void measureOneshotTimerLatencies()
{
	ros::Timer rosTimer = nodeHandle->createTimer(ros::Duration(OSTIMER_TIMEOUT_SECONDS), &timerCallback, true);
	spinUntilCallbackCalled();
	long latencyTempNs = 0;
	struct timespec startTs1;
	struct timespec startTs2;

	for(int i = 0; i < OSTIMER_TEST_RUNS; i++)
	{
		rosTimer.setPeriod(ros::Duration(OSTIMER_TIMEOUT_SECONDS));
		clock_gettime(CLOCK_ID, &startTs1);
		rosTimer.start();
		clock_gettime(CLOCK_ID, &startTs2);
		spinUntilCallbackCalled();
		struct timespec startTs;
		startTs.tv_sec = (startTs1.tv_sec + startTs2.tv_sec)/2;
		startTs.tv_nsec = (startTs1.tv_nsec + startTs2.tv_nsec)/2;
		latencyTempNs = (callbackTs.tv_sec - startTs.tv_sec) * S_TO_NS_MULTIPLIER;
		latencyTempNs += callbackTs.tv_nsec - startTs.tv_nsec;
		latencyTempNs -= OSTIMER_TIMEOUT_NANOSECONDS;
		latenciesNs[i] = latencyTempNs;
	}
}

void calcMaxAndAvgLatency()
{
	long max = latenciesNs[0];
	long min = latenciesNs[0];
	long double avg = 0.0;
	for(int i = 0; i < OSTIMER_TEST_RUNS; i++)
	{
		if(latenciesNs[i] > max)
		{
			max = latenciesNs[i];
		}
		if(latenciesNs[i] < min)
		{
			min = latenciesNs[i];
		}
		avg += (((long double)latenciesNs[i])/OSTIMER_TEST_RUNS);
	}
	maxLatencyMs = max/1000;
	minLatencyMs = min/1000;
	avgLatencyMs = avg/1000;
}

TEST_F(RosOneShotTimerTest, MaxLatencySmallerThan400MicroSecond)
{
	ASSERT_LE(maxLatencyMs, 400);
}

TEST_F(RosOneShotTimerTest, AverageLatencySmallerThan400MicroSecond)
{
	ASSERT_LE(avgLatencyMs, 400);
}
