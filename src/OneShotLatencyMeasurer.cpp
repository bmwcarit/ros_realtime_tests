#include "OneShotLatencyMeasurer.h"

#include <time.h>

#include "TestParams.h"
#include "Logger.h"

OneShotLatencyMeasurer::OneShotLatencyMeasurer(const int loopLength, double timeoutSeconds, ros::NodeHandle* nodeHandle) :
		loopLength(loopLength),
		timeoutSeconds(timeoutSeconds),
		timeoutNanoseconds(timeoutSeconds * SEC_TO_NANOSEC_MULTIPLIER),
		minLatencyNs(0),
		maxLatencyNs(0),
		avgLatencyNs(0),
		nodeHandle(nodeHandle),
		latenciesNs((long*) malloc(sizeof(long)*loopLength))
{
}

bool callbackCalled = false;
struct timespec callbackTs;
void timerCallback(const ros::TimerEvent&);
void spinUntilCallbackCalled();

void OneShotLatencyMeasurer::measure()
{
	measureOneshotTimerLatencies();
	calcMinMaxAndAvg();
}

void OneShotLatencyMeasurer::measureOneshotTimerLatencies()
{
	ros::Timer rosTimer = nodeHandle->createTimer(ros::Duration(timeoutSeconds), &timerCallback, true);
	spinUntilCallbackCalled();
	long latencyTempNs = 0;
	struct timespec startTs;

	for(int i = 0; i < loopLength; i++)
	{
		rosTimer.setPeriod(ros::Duration(timeoutSeconds));
		clock_gettime(CLOCK_ID, &startTs);
		rosTimer.start();
		spinUntilCallbackCalled();
		latencyTempNs = (callbackTs.tv_sec - startTs.tv_sec) * SEC_TO_NANOSEC_MULTIPLIER;
		latencyTempNs += callbackTs.tv_nsec - startTs.tv_nsec;
		latencyTempNs -= timeoutNanoseconds;
		latenciesNs[i] = latencyTempNs;
	}
}

void OneShotLatencyMeasurer::calcMinMaxAndAvg()
{
	long max = latenciesNs[0];
	long min = latenciesNs[0];
	long double avg = 0.0;
	for(int i = 0; i < loopLength; i++)
	{
		if(latenciesNs[i] > max)
		{
			max = latenciesNs[i];
		}
		if(latenciesNs[i] < min)
		{
			min = latenciesNs[i];
		}
		avg += (((long double)latenciesNs[i])/loopLength);
	}
	maxLatencyNs = max;
	minLatencyNs = min;
	avgLatencyNs = avg;
}

void OneShotLatencyMeasurer::printMeasurementResults()
{
	std::stringstream ss;
	ss << "Measurement results with a loop length of " << loopLength << " and a timeout of " << (int) (timeoutNanoseconds/1000) << " us:";
	Logger::INFO(ss.str().c_str());
	ss.str("");
	ss <<"MIN:  " << getMinLatencyMs() << "us \tAVG:  " << getAvgLatencyMs() << "us \tMAX:  " << getMaxLatencyMs() << "us";
	Logger::INFO(ss.str().c_str());
}

int OneShotLatencyMeasurer::getMaxLatencyMs()
{
	return maxLatencyNs/1000;
}

int OneShotLatencyMeasurer::getMinLatencyMs()
{
	return minLatencyNs/1000;
}

int OneShotLatencyMeasurer::getAvgLatencyMs()
{
	return avgLatencyNs/1000;
}

void timerCallback(const ros::TimerEvent&)
{
	clock_gettime(CLOCK_ID, &callbackTs);
	callbackCalled = true;
}

void spinUntilCallbackCalled()
{
	callbackCalled = false;
	while(!callbackCalled)
	{
		ros::spinOnce();
	}
}

OneShotLatencyMeasurer::~OneShotLatencyMeasurer()
{
	delete latenciesNs;
}

