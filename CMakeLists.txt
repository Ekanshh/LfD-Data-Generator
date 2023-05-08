cmake_minimum_required(VERSION 2.8.3)
project(lfd_data_generator)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  src/generator.py
  src/recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
