#ifndef ONESHOTLATENCYMEASURER_H_
#define ONESHOTLATENCYMEASURER_H_

#include "ros/ros.h"
#include <string>
#include <time.h>

class OneShotLatencyMeasurer {
public:
	OneShotLatencyMeasurer(const int loopLength, long timeoutNanoSeconds, ros::NodeHandle* nodeHandle, bool lockMemory);
	~OneShotLatencyMeasurer();
	void measure();
	void printMeasurementResults();
	void saveDiffGPlotData(std::string filename);
	void saveMeasuredLatencyGPlotData(std::string filename);
	void saveReportedLatencyGPlotData(std::string filename);
	const static int clock_id = CLOCK_MONOTONIC_RAW;

	//Getter methods
	int getMaxLatencyUs();
	int getMinLatencyUs();
	int getAvgLatencyUs();
	int getMaxReportedLatencyUs();
	int getMinReportedLatencyUs();
	int getAvgReportedLatencyUs();
	int getMaxDifferenceUs();
	int getMinDifferenceUs();
	int getAvgDifferenceAbsUs();
private:
	const int loopLength;
	const double timeoutSeconds;
	const long timeoutNanoseconds;
	long minLatencyNs;
	long maxLatencyNs;
	unsigned long long avgLatencyNs;
	long minLatencyReportedNs;
	long maxLatencyReportedNs;
	unsigned long long avgLatencyReportedNs;
	ros::NodeHandle* nodeHandle;
	long* latenciesNs;
	long* latenciesReportedNs;
	long* differenceNs;
	bool callbackCalled;
	struct timespec callbackTs;
	int loopCounter;
	long maxDifference;
	long minDifference;
	unsigned long long avgDifferenceAbs;
	const bool lockMemory;

	void saveGPlotData(std::string filename, long* plotValues, int maxValueMs, int minValueMs);
	void measureOneshotTimerLatencies();
	void calcMinMaxAndAvg();
	void timerCallback(const ros::TimerEvent&);
	void spinUntilCallbackCalled();
};

#endif //ONESHOTLATENCYMEASURER_H_

