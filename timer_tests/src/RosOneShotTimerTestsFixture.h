#ifndef ROSONESHOTTIMERTESTSFIXTURE_H_
#define ROSONESHOTTIMERTESTSFIXTURE_H_

#include <gtest/gtest.h>

class RosOneShotTimerTests : public ::testing::Test {
protected:
	static int minLatencyMs;
	static int maxLatencyMs;
	static int avgLatencyMs;
	static bool setupSucceeded;

	static void SetUpTestCase();

	virtual void SetUp();
};

#endif //ROSONESHOTTIMERTESTSFIXTURE_H_

