cmake_minimum_required(VERSION 2.8.3)

project(openni_tracker_torreta)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS geometry_msgs
					roscpp
					roslib)
# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

catkin_package()

include_directories(${catkin_INCLUDEDIR}
		    ${OpenNI_INCLUDEDIR})

link_directories(${catkin_LIBRARY_DIRS})

add_executable(openni_tracker_torreta src/openni_tracker_torreta.cpp)

add_dependencies(openni_tracker_torreta geometry_msgs_gencpp)

target_link_libraries(openni_tracker_torreta ${catkin_LIBRARIES}
				     ${OpenNI_LIBRARIES}
				     ${orocos_kdl_LIBRARIES})

install(TARGETS openni_tracker_torreta RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES openni_tracker_torreta.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
