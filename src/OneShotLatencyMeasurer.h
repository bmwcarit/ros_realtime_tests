#ifndef ONESHOTLATENCYMEASURER_H_
#define ONESHOTLATENCYMEASURER_H_

#include "ros/ros.h"
#include <string>
#include <time.h>

class OneShotLatencyMeasurer {
public:
	OneShotLatencyMeasurer(const int loopLength, long timeoutNanoSeconds, ros::NodeHandle* nodeHandle);
	~OneShotLatencyMeasurer();
	void measure();
	void printMeasurementResults();
	int getMaxLatencyMs();
	int getMinLatencyMs();
	int getAvgLatencyMs();
	int getMaxReportedLatencyMs();
	int getMinReportedLatencyMs();
	int getAvgReportedLatencyMs();
	void saveGPlotData(std::string filename);
private:
	const int loopLength;
	const double timeoutSeconds;
	const long timeoutNanoseconds;
	long minLatencyNs;
	long maxLatencyNs;
	long avgLatencyNs;
	long minLatencyReportedNs;
	long maxLatencyReportedNs;
	long avgLatencyReportedNs;
	ros::NodeHandle* nodeHandle;
	long* latenciesNs;
	long* latenciesReportedNs;
	bool callbackCalled;
	struct timespec callbackTs;
	int loopCounter;

	void measureOneshotTimerLatencies();
	void calcMinMaxAndAvg();
	void timerCallback(const ros::TimerEvent&);
	void spinUntilCallbackCalled();
};

#endif //ONESHOTLATENCYMEASURER_H_

