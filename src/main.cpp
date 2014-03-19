#include "main.h"
#include "Logger.h"
#include "TestParams.h"

ros::NodeHandle* nodeHandle;
struct timespec callbackTs;
bool callbackCalled = false;


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Timer_tests");
	testing::InitGoogleTest(&argc, argv);
	nodeHandle = new ros::NodeHandle;
	return RUN_ALL_TESTS();
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
