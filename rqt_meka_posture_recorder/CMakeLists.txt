cmake_minimum_required(VERSION 2.8.3)
project(rqt_meka_posture_recorder)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(DIRECTORY ui
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
   scripts/rqt_meka_posture_recorder
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


