cmake_minimum_required(VERSION 2.8.3)
project(oneshot_timer_tests)

find_package(catkin REQUIRED COMPONENTS roscpp rt_tests_support)

catkin_package(CATKIN_DEPENDS rt_tests_support)

include_directories(${catkin_INCLUDE_DIRS} ../rt_tests_support/include)

add_executable(oneshot_timer_tests
   ./src/main.cpp
   ./src/Config.cpp
   ./src/ArgumentParser.cpp
   ./src/OneShotLatencyMeasurer.cpp
   ./src/BusyOneShotLatencyMeasurer.cpp
   ./src/IdleOneShotLatencyMeasurer.cpp
)
target_link_libraries(oneshot_timer_tests ${catkin_LIBRARIES})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS oneshot_timer_tests
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
