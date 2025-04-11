cmake_minimum_required(VERSION 3.0.2)
project(coffee_maker)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	message_generation
	baxter_interface
)

catkin_python_setup()

generate_messages(DEPENDENCIES
  std_msgs
)

## Build depends
catkin_package(CATKIN_DEPENDS
	rospy
	std_msgs
	baxter_interface
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS 
	
  	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

