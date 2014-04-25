cmake_minimum_required(VERSION 2.8.3)
project(timer_tests)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ../gtest/include ../gtest ../rt_tests_support/include)

add_library(gtest_static STATIC ../gtest/src/gtest-all.cc)
add_library(rt_tests_support STATIC ../rt_tests_support/src/Logger.cpp ../rt_tests_support/src/PrioritySwitcher.cpp ../rt_tests_support/src/MeasurementDataEvaluator.cpp)

add_executable(system_tests ./src/main.cpp ./src/SystemTests.cpp)
target_link_libraries(system_tests gtest_static rt_tests_support ${catkin_LIBRARIES})

add_executable(timer_tests ./src/main.cpp ./src/OneShotLatencyMeasurer.cpp ./src/RosOneShotTimerTestsFixture.cpp ./src/OneShotTimerTests.cpp)
target_link_libraries(timer_tests gtest_static rt_tests_support ${catkin_LIBRARIES})

install(TARGETS system_tests timer_tests
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
