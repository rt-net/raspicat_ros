cmake_minimum_required(VERSION 3.0.2)
project(raspicat_control)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

#############
## Install ##
#############


# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()