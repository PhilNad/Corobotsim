cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

if(Boost_FOUND)
	include_directories(${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} "/usr/include/python2.7")
	link_directories(${GAZEBO_LIBRARY_DIRS})
	list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

	set(Boost_USE_STATIC_LIBS OFF)
	set(Boost_USE_MULTITHREADED ON)
	set(Boost_USE_STATIC_RUNTIME OFF)

	find_package(Boost COMPONENTS python)

	#Generate this shared libary as an output
	add_library(GazeboInterface SHARED GazeboInterface.cpp)

	#Links againsts these libraries
	target_link_libraries(GazeboInterface ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} pthread python2.7 boost_python)

	#Rename the file
	add_custom_command(TARGET GazeboInterface POST_BUILD COMMAND ${CMAKE_COMMAND} -E rename "$<TARGET_FILE:GazeboInterface>" "${CMAKE_BINARY_DIR}/ForceTorqueSensorInterface.so")

	#This post build commande copies the resulting object into python libraries folder
	add_custom_command(TARGET GazeboInterface POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_BINARY_DIR}/ForceTorqueSensorInterface.so" "/home/pnadeau/catkin_ws/devel/lib/python2.7/dist-packages/")
elseif(NOT Boost_FOUND)
	message(FATAL_ERROR "Unable to find correct Boost version. Did you set BOOST_ROOT?")
endif()
