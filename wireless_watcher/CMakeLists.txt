cmake_minimum_required(VERSION 2.8.3)
project(wireless_watcher)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(PROGRAMS
  scripts/watcher_node
  scripts/mikrotik_watcher_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint)
  find_package(catkin REQUIRED COMPONENTS roslaunch)
  roslaunch_add_file_check(launch)
endif()
