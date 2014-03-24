#include "OneShotLatencyMeasurer.h"

#include <time.h>
#include <fstream>
#include <iomanip>

#include "TestParams.h"
#include "Logger.h"

OneShotLatencyMeasurer::OneShotLatencyMeasurer(const int loopLength, long timeoutNanoSeconds, ros::NodeHandle* nodeHandle) :
		loopLength(loopLength),
		timeoutSeconds(((double)timeoutNanoSeconds)/SEC_TO_NANOSEC_MULTIPLIER),
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
	ros::Timer rosTimer = nodeHandle->createTimer(ros::Duration(0.01), timerCallback, true);
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

void OneShotLatencyMeasurer::saveGPlotData(std::string filename)
{
	const int latHitArraySize = getMaxLatencyMs() + 1;
	int hits[latHitArraySize];
	for(int i = 0; i < latHitArraySize; i++)
	{
		hits[i] = 0;
	}

	for(int i = 0; i < loopLength; i++)
	{
		if(latenciesNs[i] >= 0)
		{
			hits[latenciesNs[i]/1000]++;
		}
	}

	std::ofstream fs;
	fs.open(filename.c_str());
	fs << "# Latency Plot data for GnuPlot" << std::endl;
	fs << "# Timeout: " << (int) timeoutNanoseconds/1000 << "us, LoopLength: " << loopLength << std::endl;
	fs << "# MIN: " << getMinLatencyMs()  << "us \tAVG: " << getAvgLatencyMs() << "us \tMAX: " << getMaxLatencyMs() << "us" << std::endl;
	for(int i = 0; i < latHitArraySize; i++)
	{
		fs << std::setfill('0') << std::setw(6) << i << " \t" << std::setfill('0') << std::setw(6) << hits[i] << std::endl;
	}

	if(getMinLatencyMs() < 0)
	{
		const int negHitArraySize = getMinLatencyMs()*(-1) + 1;
		fs << "# negative Latencies following" << std::endl;
		int negHits[negHitArraySize];
		for(int i = 0; i < negHitArraySize; i++)
		{
			negHits[i] = 0;
		}
		for(int i = 0; i < loopLength; i++)
		{
			if(latenciesNs[i] < 0)
			{
				negHits[(latenciesNs[i] * (-1))/1000]++;
			}
		}

		for(int i = 1; i < negHitArraySize; i++)
		{
			fs << "-" << std::setfill('0') << std::setw(6) << i << " \t" << std::setfill('0') << std::setw(6) << negHits[i] << std::endl;
		}
	}

	fs << std::endl;
	fs.close();
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
	while(!callbackCalled && ros::ok())
	{
		ros::spinOnce();
	}
}

OneShotLatencyMeasurer::~OneShotLatencyMeasurer()
{
	delete latenciesNs;
}

