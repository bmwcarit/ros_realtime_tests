#ifndef ONESHOTLATENCYMEASURER_H_
#define ONESHOTLATENCYMEASURER_H_

#include "ros/ros.h"

class OneShotLatencyMeasurer {
public:
	OneShotLatencyMeasurer(const int loopLength, double timeoutSeconds, ros::NodeHandle* nodeHandle);
	~OneShotLatencyMeasurer();
	void measure();
	void printMeasurementResults();
	int getMaxLatencyMs();
	int getMinLatencyMs();
	int getAvgLatencyMs();
private:
	const int loopLength;
	const double timeoutSeconds;
	const long timeoutNanoseconds;
	long minLatencyNs;
	long maxLatencyNs;
	long avgLatencyNs;
	ros::NodeHandle* nodeHandle;
	long* latenciesNs;

	void measureOneshotTimerLatencies();
	void calcMinMaxAndAvg();
};

#endif //ONESHOTLATENCYMEASURER_H_

