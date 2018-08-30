cmake_minimum_required(VERSION 2.8.3)
project(wireless_watcher)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  scripts/watcher_node
  scripts/mikrotik_watcher_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
