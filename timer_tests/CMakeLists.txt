cmake_minimum_required(VERSION 2.8.3)
project(timer_tests)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ./gtest/include ./gtest)

add_library(gtest_static STATIC ./gtest/src/gtest-all.cc)

set(COMMON_SOURCES ./src/main.cpp ./src/Logger.cpp ./src/OneShotLatencyMeasurer.cpp ./src/PrioritySwitcher.cpp ./src/RosOneShotTimerTestsFixture.cpp)

add_executable(system_tests ./src/main.cpp ./src/Logger.cpp ./src/PrioritySwitcher.cpp ./src/SystemTests.cpp)
target_link_libraries(system_tests gtest_static pthread ${catkin_LIBRARIES})

add_executable(timer_tests ${COMMON_SOURCES} ./src/OneShotTimerTests.cpp)
target_link_libraries(timer_tests gtest_static pthread ${catkin_LIBRARIES})

install(TARGETS system_tests timer_tests
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
