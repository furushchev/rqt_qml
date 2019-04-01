cmake_minimum_required(VERSION 2.8.3)

project(rqt_qml)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/rqt_qml
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch lib plugins qml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
