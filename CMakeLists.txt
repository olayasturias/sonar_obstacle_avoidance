cmake_minimum_required(VERSION 3.0.2)
project(sonar_obstacle_avoidance)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
   nodes/naive_avoidance.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)