cmake_minimum_required(VERSION 2.8.3)
project(cyclic_timer_tests)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ../rt_tests_support/include)

add_library(rt_tests_support STATIC
   ../rt_tests_support/src/Logger.cpp
   ../rt_tests_support/src/PrioritySwitcher.cpp
   ../rt_tests_support/src/PlotDataFileCreator.cpp
   ../rt_tests_support/src/MeasurementDataEvaluator.cpp
)

add_executable(cyclic_timer_tests
   ./src/main.cpp
   ./src/Config.cpp
   ./src/ArgumentParser.cpp
   ./src/CyclicLatencyMeasurer.cpp
)
target_link_libraries(cyclic_timer_tests rt_tests_support ${catkin_LIBRARIES})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS cyclic_timer_tests
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
