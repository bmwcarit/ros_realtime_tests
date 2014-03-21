#ifndef ONESHOTLATENCYMEASURER_H_
#define ONESHOTLATENCYMEASURER_H_

#include "ros/ros.h"
#include <string>

class OneShotLatencyMeasurer {
public:
	OneShotLatencyMeasurer(const int loopLength, long timeoutNanoSeconds, ros::NodeHandle* nodeHandle);
	~OneShotLatencyMeasurer();
	void measure();
	void printMeasurementResults();
	int getMaxLatencyMs();
	int getMinLatencyMs();
	int getAvgLatencyMs();
	void saveGPlotData(std::string filename);
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

