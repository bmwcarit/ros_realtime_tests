cmake_minimum_required(VERSION 2.8.3)
project(timer_tests)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES timer_tests
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${gtest_SOURCE_DIR}/include
  ${gtest_SOURCE_DIR}
)

add_library(jonastest STATIC ${gtest_SOURCE_DIR}/src/gtest-all.cc)
set_target_properties(jonastest PROPERTIES COMPILEFLAGS "")

add_executable(timer_tests ./src/main.cpp)
target_link_libraries(timer_tests jonastest pthread)

