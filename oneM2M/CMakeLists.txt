cmake_minimum_required(VERSION 2.8.3)
project(onem2m)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

install(PROGRAMS
   scripts/app.js
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

