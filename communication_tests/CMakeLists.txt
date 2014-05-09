cmake_minimum_required(VERSION 2.8.3)
project(communication_tests)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

add_message_files(FILES timestamp_msg.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS} ../rt_tests_support/include)

add_library(rt_tests_support STATIC
   ../rt_tests_support/src/Logger.cpp
   ../rt_tests_support/src/PrioritySwitcher.cpp
   ../rt_tests_support/src/PlotDataFileCreator.cpp
   ../rt_tests_support/src/MeasurementDataEvaluator.cpp
)

add_executable(communication_tests_publisher
   ./src/Config.cpp
   ./src/Publisher.cpp
   ./src/ArgumentParser.cpp
   ./src/ct_publisher_node.cpp
)
target_link_libraries(communication_tests_publisher rt_tests_support ${catkin_LIBRARIES})

add_executable(communication_tests_subscriber
   ./src/Config.cpp
   ./src/Subscriber.cpp
   ./src/ArgumentParser.cpp
   ./src/ct_subscriber_node.cpp
)
target_link_libraries(communication_tests_subscriber rt_tests_support ${catkin_LIBRARIES})

install(TARGETS communication_tests_publisher communication_tests_subscriber
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
