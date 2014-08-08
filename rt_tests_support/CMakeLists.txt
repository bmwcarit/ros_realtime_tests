cmake_minimum_required(VERSION 2.8.3)
project(rt_tests_support)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(DEPENDS roscpp
	       INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME})

include_directories(${catkin_INCLUDE_DIRS} ./include)

add_library(${PROJECT_NAME} STATIC
   ../rt_tests_support/src/Logger.cpp
   ../rt_tests_support/src/PrioritySwitcher.cpp
   ../rt_tests_support/src/PlotDataFileCreator.cpp
   ../rt_tests_support/src/MeasurementDataEvaluator.cpp
)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
