cmake_minimum_required(VERSION 2.8.3)
project(wireless_watcher)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

set(python_scripts
  scripts/watcher_node
  scripts/mikrotik_watcher_node
)

catkin_install_python(PROGRAMS ${python_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(roslint REQUIRED)
roslint_python(${python_scripts})
roslint_python()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  roslint_add_test()
endif()
