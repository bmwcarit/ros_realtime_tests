#ifndef ROSONESHOTTIMERTESTSFIXTURE_H_
#define ROSONESHOTTIMERTESTSFIXTURE_H_

#include <gtest/gtest.h>

class RosOneShotTimerTestsFixture : public ::testing::Test {
protected:
	static const int amountTimeouts = 3;
	static int minLatencyMs[amountTimeouts];
	static int maxLatencyMs[amountTimeouts];
	static int avgLatencyMs[amountTimeouts];
	static bool setupSucceeded;
	static bool rtState;

	static void SetUpTestCase();

	virtual void SetUp();
};

#endif //ROSONESHOTTIMERTESTSFIXTURE_H_

