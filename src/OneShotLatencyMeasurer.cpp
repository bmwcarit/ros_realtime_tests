#include "OneShotLatencyMeasurer.h"

#include <fstream>
#include <iomanip>

#include "TestParams.h"
#include "Logger.h"

OneShotLatencyMeasurer::OneShotLatencyMeasurer(const int loopLength, long timeoutNanoSeconds, ros::NodeHandle* nodeHandle) :
		loopLength(loopLength),
		timeoutSeconds(((double)timeoutNanoSeconds)/SEC_TO_NANOSEC_MULTIPLIER),
		timeoutNanoseconds(timeoutSeconds * SEC_TO_NANOSEC_MULTIPLIER),
		minLatencyNs(0), maxLatencyNs(0), avgLatencyNs(0),
		minLatencyReportedNs(0), maxLatencyReportedNs(0), avgLatencyReportedNs(0),
		nodeHandle(nodeHandle),
		latenciesNs((long*) malloc(sizeof(long)*loopLength)),
		latenciesReportedNs((long*) malloc(sizeof(long)*loopLength)),
		differenceNs((long*) malloc(sizeof(long)*loopLength)),
		callbackCalled(false),
		callbackTs(),
		loopCounter(0),
		maxDifference(0), minDifference(0), avgDifference(0)
{
}

void OneShotLatencyMeasurer::measure()
{
	measureOneshotTimerLatencies();
	calcMinMaxAndAvg();
}

void OneShotLatencyMeasurer::measureOneshotTimerLatencies()
{
	loopCounter = 0;
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
		latencyTempNs = ((callbackTs.tv_sec - startTs.tv_sec) * SEC_TO_NANOSEC_MULTIPLIER) + (callbackTs.tv_nsec - startTs.tv_nsec);
		latencyTempNs -= timeoutNanoseconds;
		latenciesNs[loopCounter] = latencyTempNs;
	}
}

void OneShotLatencyMeasurer::calcMinMaxAndAvg()
{
	maxLatencyNs = latenciesNs[0];
	minLatencyNs = latenciesNs[0];
	avgLatencyNs = 0.0;
	maxLatencyReportedNs = latenciesReportedNs[0];
	minLatencyReportedNs = latenciesReportedNs[0];
	avgLatencyReportedNs = 0.0;
	maxDifference = latenciesNs[0] - latenciesReportedNs[0];
	minDifference = latenciesNs[0] - latenciesReportedNs[0];
	avgDifference = 0.0;
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
			maxLatencyReportedNs = latenciesReportedNs[i];
		}
		if(latenciesReportedNs[i] < minLatencyReportedNs)
		{
			minLatencyReportedNs = latenciesReportedNs[i];
		}
		avgLatencyNs += latenciesNs[i];
		avgLatencyReportedNs += latenciesReportedNs[i];
		differenceNs[i] = latenciesReportedNs[i] - latenciesNs[i];
		if(differenceNs[i] > maxDifference)
		{
			maxDifference = differenceNs[i];
		}
		if(differenceNs[i] < minDifference)
		{
			minDifference = differenceNs[i];
		}
		avgDifference += differenceNs[i];
	}
	avgLatencyNs /= loopLength;
	avgLatencyReportedNs /= loopLength;
	avgDifference /= loopLength;
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
	ss.str("");
	ss << "Difference (reported-measured):\tMIN: " << getMinDifferenceMs() << "us\tAVG_ABS: " << getAvgDifferenceMs() << "us\tMAX: " << getMaxDifferenceMs() << "us";
	Logger::INFO(ss.str().c_str());
}

void OneShotLatencyMeasurer::saveMeasuredLatencyGPlotData(std::string filename)
{
	saveGPlotData(filename, latenciesNs, getMaxLatencyMs(), getMinLatencyMs());
}

void OneShotLatencyMeasurer::saveReportedLatencyGPlotData(std::string filename)
{
	saveGPlotData(filename, latenciesReportedNs, getMaxReportedLatencyMs(), getMinReportedLatencyMs());
}
void OneShotLatencyMeasurer::saveDiffGPlotData(std::string filename)
{
	saveGPlotData(filename, differenceNs, getMaxDifferenceMs(), getMinDifferenceMs());
}

void OneShotLatencyMeasurer::saveGPlotData(std::string filename, long* plotValues, int maxValueMs, int minValueMs)
{
	const int latHitArraySize = maxValueMs + 1;
	int hits[latHitArraySize];
	for(int i = 0; i < latHitArraySize; i++)
	{
		hits[i] = 0;
	}

	for(int i = 0; i < loopLength; i++)
	{
		if(plotValues[i] >= 0)
		{
			hits[plotValues[i]/1000]++;
		}
	}

	std::ofstream fs;
	fs.open(filename.c_str());
	fs << "# Plot data for GnuPlot" << std::endl;
	fs << "# Timeout: " << (int) timeoutNanoseconds/1000 << "us, LoopLength: " << loopLength << std::endl;
	fs << "# Measured:\t MIN: " << getMinLatencyMs()  << "us \tAVG: " << getAvgLatencyMs() << "us \tMAX: " << getMaxLatencyMs() << "us" << std::endl;
	fs << "# Reported:\t MIN: " << getMinReportedLatencyMs()  << "us \tAVG: " << getAvgReportedLatencyMs() << "us \tMAX: " << getMaxReportedLatencyMs() << "us" << std::endl;
	fs << "# Difference:\t MIN: " << getMinDifferenceMs()  << "us \tAVG: " << getAvgDifferenceMs() << "us \tMAX: " << getMaxDifferenceMs() << "us" << std::endl;
	for(int i = 0; i < latHitArraySize; i++)
	{
		fs << std::setfill('0') << std::setw(6) << i << " \t" << std::setfill('0') << std::setw(6) << hits[i] << std::endl;
	}

	if(minValueMs < 0)
	{
		const int negHitArraySize = minValueMs*(-1) + 1;
		fs << "# negative Values following" << std::endl;
		int negHits[negHitArraySize];
		for(int i = 0; i < negHitArraySize; i++)
		{
			negHits[i] = 0;
		}
		for(int i = 0; i < loopLength; i++)
		{
			if(plotValues[i] < 0)
			{
				negHits[(plotValues[i] * (-1))/1000]++;
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

int OneShotLatencyMeasurer::getMaxDifferenceMs()
{
	return maxDifference/1000;
}

int OneShotLatencyMeasurer::getMinDifferenceMs()
{
	return minDifference/1000;
}

int OneShotLatencyMeasurer::getAvgDifferenceMs()
{
	return avgDifference/1000;
}

void OneShotLatencyMeasurer::timerCallback(const ros::TimerEvent& te)
{
	clock_gettime(CLOCK_ID, &callbackTs);
	latenciesReportedNs[loopCounter] = ((te.current_real.sec - te.current_expected.sec) * SEC_TO_NANOSEC_MULTIPLIER) + (te.current_real.nsec - te.current_expected.nsec);
	if(latenciesReportedNs[loopCounter] > 1000000000)
	{
		std::stringstream ss;
		ss << "UNREALISTICALLY HIGH LATENCY: " << latenciesReportedNs[loopCounter];
		Logger::ERROR(ss.str());
		ss.str("");
		ss << "expected.sec: " << te.current_expected.sec << "   expected.nsec: " << te.current_expected.nsec;
		Logger::ERROR(ss.str());
		ss.str("");
		ss << "real.sec: " << te.current_real.sec << "   real.nsec: " << te.current_real.nsec;
		Logger::ERROR(ss.str());
	}
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
	delete differenceNs;
	delete latenciesReportedNs;
	delete latenciesNs;
}

