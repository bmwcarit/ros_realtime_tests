cmake_minimum_required(VERSION 2.8.3)
project(communication_tests)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rt_tests_support)

add_message_files(FILES timestamp_msg.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime rt_tests_support)

include_directories(${catkin_INCLUDE_DIRS} ../rt_tests_support/include)

add_executable(communication_tests_publisher
   ./src/Config.cpp
   ./src/Publisher.cpp
   ./src/ArgumentParser.cpp
   ./src/ct_publisher_node.cpp
)
add_dependencies(communication_tests_publisher ${PROJECT_NAME}_gencpp)
target_link_libraries(communication_tests_publisher ${catkin_LIBRARIES})

add_executable(communication_tests_subscriber
   ./src/Config.cpp
   ./src/Subscriber.cpp
   ./src/ArgumentParser.cpp
   ./src/ct_subscriber_node.cpp
)
add_dependencies(communication_tests_subscriber ${PROJECT_NAME}_gencpp)
target_link_libraries(communication_tests_subscriber ${catkin_LIBRARIES})

add_executable(bypass_node
   ./src/bypass_node.cpp
)
add_dependencies(bypass_node ${PROJECT_NAME}_gencpp)
target_link_libraries(bypass_node ${catkin_LIBRARIES})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS communication_tests_publisher communication_tests_subscriber bypass_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
