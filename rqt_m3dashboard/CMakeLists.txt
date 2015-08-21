cmake_minimum_required(VERSION 2.8.3)
project(rqt_m3dashboard)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
   scripts/rqt_m3dashboard
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach (dir launch config images)
   install(DIRECTORY ${dir}/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
