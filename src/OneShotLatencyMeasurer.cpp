#include "OneShotLatencyMeasurer.h"

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
		minLatencyReportedNs(0),
		maxLatencyReportedNs(0),
		avgLatencyReportedNs(0),
		nodeHandle(nodeHandle),
		latenciesNs((long*) malloc(sizeof(long)*loopLength)),
		latenciesReportedNs((long*) malloc(sizeof(long)*loopLength)),
		callbackCalled(false),
		callbackTs(),
		loopCounter(0)
{
}

void OneShotLatencyMeasurer::measure()
{
	measureOneshotTimerLatencies();
	calcMinMaxAndAvg();
}

void OneShotLatencyMeasurer::measureOneshotTimerLatencies()
{
	ros::Timer rosTimer = nodeHandle->createTimer(ros::Duration(0.01), &OneShotLatencyMeasurer::timerCallback, this, true);
	spinUntilCallbackCalled();
	long latencyTempNs = 0;
	struct timespec startTs;

	for(loopCounter = 0; loopCounter < loopLength; loopCounter++)
	{
		rosTimer.setPeriod(ros::Duration(timeoutSeconds));
		clock_gettime(CLOCK_ID, &startTs);
		rosTimer.start();
		spinUntilCallbackCalled();
		latencyTempNs = (callbackTs.tv_sec - startTs.tv_sec) * SEC_TO_NANOSEC_MULTIPLIER;
		latencyTempNs += callbackTs.tv_nsec - startTs.tv_nsec;
		latencyTempNs -= timeoutNanoseconds;
		latenciesNs[loopCounter] = latencyTempNs;
	}
}

void OneShotLatencyMeasurer::calcMinMaxAndAvg()
{
	maxLatencyNs = latenciesNs[0];
	minLatencyNs = latenciesNs[0];
	long double avg = 0.0;
	maxLatencyReportedNs = latenciesReportedNs[0];
	minLatencyReportedNs = latenciesReportedNs[0];
	long double avgRep = 0.0;
	for(int i = 0; i < loopLength; i++)
	{
		if(latenciesNs[i] > maxLatencyNs)
		{
			maxLatencyNs = latenciesNs[i];
		}
		if(latenciesNs[i] < minLatencyNs)
		{
			minLatencyNs = latenciesNs[i];
		}
		if(latenciesReportedNs[i] > maxLatencyReportedNs)
		{
			maxLatencyReportedNs = latenciesNs[i];
		}
		if(latenciesReportedNs[i] < minLatencyReportedNs)
		{
			minLatencyReportedNs = latenciesNs[i];
		}
		avg += (((long double)latenciesNs[i])/loopLength);
		avgRep += (((long double)latenciesReportedNs[i])/loopLength);
	}
	avgLatencyNs = avg;
	avgLatencyReportedNs = avgRep;
}

void OneShotLatencyMeasurer::printMeasurementResults()
{
	std::stringstream ss;
	ss << "Measurement results with a loop length of " << loopLength << " and a timeout of " << (int) (timeoutNanoseconds/1000) << " us:";
	Logger::INFO(ss.str().c_str());
	ss.str("");
	ss <<"Measured:\tMIN:  " << getMinLatencyMs() << "us \tAVG:  " << getAvgLatencyMs() << "us \tMAX:  " << getMaxLatencyMs() << "us";
	Logger::INFO(ss.str().c_str());
	ss.str("");
	ss <<"Reported:\tMIN:  " << getMinReportedLatencyMs() << "us \tAVG:  " << getAvgReportedLatencyMs() << "us \tMAX:  " << getMaxReportedLatencyMs() << "us";
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

int OneShotLatencyMeasurer::getMaxReportedLatencyMs()
{
	return maxLatencyReportedNs/1000;
}

int OneShotLatencyMeasurer::getMinReportedLatencyMs()
{
	return minLatencyReportedNs/1000;
}

int OneShotLatencyMeasurer::getAvgReportedLatencyMs()
{
	return avgLatencyReportedNs/1000;
}

void OneShotLatencyMeasurer::timerCallback(const ros::TimerEvent& te)
{
	clock_gettime(CLOCK_ID, &callbackTs);
	latenciesReportedNs[loopCounter] = SEC_TO_NANOSEC_MULTIPLIER * (te.current_expected.sec - te.current_real.sec);
	latenciesReportedNs[loopCounter] += (te.current_expected.nsec - te.current_real.nsec);
	callbackCalled = true;
}

void OneShotLatencyMeasurer::spinUntilCallbackCalled()
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

