#include "ros/ros.h"

#include <gtest/gtest.h>

int main(int argc, char* argv[])
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

TEST(TimerTests, initialTestcase)
{
	ASSERT_TRUE(1==1);
}
