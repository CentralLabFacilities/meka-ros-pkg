cmake_minimum_required(VERSION 2.8.3)
project(meka_velocity_smoother)
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib nodelet dynamic_reconfigure geometry_msgs nav_msgs)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/params.cfg)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}_nodelet
   CATKIN_DEPENDS roscpp pluginlib nodelet ecl_threads dynamic_reconfigure geometry_msgs nav_msgs
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

INCLUDE(FindPkgConfig)
##libreflexxes
PKG_CHECK_MODULES(REFLEXXES REQUIRED libReflexxesTypeII>=1.2.3)
IF (NOT REFLEXXES_FOUND)
  message(FATAL_ERROR "Error: could not find lib libReflexxesTypeII")
ENDIF ()
SET(REFLEXXES_LIBRARY_DIRS "${REFLEXXES_PREFIX}/lib")
SET(REFLEXXES_INCLUDE_DIRS "${REFLEXXES_PREFIX}/include")
#resolve to absolute library filename
#or is there an easier way to get the absolute lib filename from pkg config files?!
find_library(REFLEXXES_LIBRARY NAMES ${REFLEXXES_LIBRARIES} HINTS ${REFLEXXES_LIBRARY_DIRS})
MESSAGE("-- using libReflexxesTypeII version ${REFLEXXES_VERSION} from ${REFLEXXES_INCLUDE_DIRS} and ${REFLEXXES_LIBRARY_DIRS}")
MESSAGE("-- will link against ${REFLEXXES_LIBRARY}")


include_directories(include ${REFLEXXES_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


# Nodelet library
add_library(${PROJECT_NAME}_nodelet src/velocity_smoother_nodelet.cpp)

set_property(TARGET ${PROJECT_NAME}_nodelet PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)


target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${REFLEXXES_LIBRARY})
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)


install(TARGETS ${PROJECT_NAME}_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
