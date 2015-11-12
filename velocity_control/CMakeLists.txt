cmake_minimum_required(VERSION 2.8)
project(velocity_control CXX C)

set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "7")
set(CPACK_PACKAGE_VERSION_PATCH "0")

set(APP_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(APP_ASSOCIATED_TAG "v${APP_VERSION}")

set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")

#find_package(Boost REQUIRED COMPONENTS system regex)
#find_package(<package> REQUIRED)
find_package(Threads)

#include_directories(${Boost_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARY_DIRS})
#set(LIBS ${LIBS} ${Boost_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	roscpp
	message_generation
)

find_package( wiringPi REQUIRED )

#find_package(SFML REQUIRED system window graphics)
find_package(simplified_ros REQUIRED)


#Add header files
include_directories(
	${catkin_INCLUDE_DIRS}
	${simplified_ros_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/include)

#Add a library
#add_library(<lib> SHARED <source>)
#target_link_libraries(<lib> ${LIBS})
#set(LIBS <lib> ${LIBS})
#set_target_properties(<lib> PROPERTIES VERSION ${APP_VERSION}) #To apply the version number

#Generate messages in the 'msg' folder
add_message_files(
	FILES
	command.msg
	event.msg
)

#Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs  # Or other packages containing msgs
)

#Catkin specific configuration
catkin_package(
	DEPENDS message_runtime
)

include_directories(${CATKIN_INCLUDE_DIRS} ${wiringPi_INCLUDE_DIRS})

# Find SFML (for Gamepad control)
# You can find FindSFML.cmake here:
# https://raw.githubusercontent.com/SFML/SFML/master/cmake/Modules/FindSFML.cmake

#include_directories(${SFML_INCLUDE_DIR})
#
#add_executable(velocity_control_remote_commander src/velocity_control_remote_commander.cpp)
#target_link_libraries(velocity_control_remote_commander ${catkin_LIBRARIES} ${simplified_ros_LIBRARIES} ${SFML_LIBRARIES})
#set_target_properties(velocity_control_remote_commander PROPERTIES VERSION ${APP_VERSION})


#Add an executable
add_executable(velocity_control src/velocity_control.cpp)
target_link_libraries(velocity_control ${catkin_LIBRARIES} ${simplified_ros_LIBRARIES} ${wiringPi_LIBRARIES})
set_target_properties(velocity_control PROPERTIES VERSION ${APP_VERSION})

add_executable(velocity_control_tester src/velocity_control_tester.cpp)
target_link_libraries(velocity_control_tester ${catkin_LIBRARIES} ${simplified_ros_LIBRARIES})
set_target_properties(velocity_control_tester PROPERTIES VERSION ${APP_VERSION})



#TODO Create an automatic test if feasible in ros
#add_executable(test_state_machine src/test_state_machine.cpp)
#target_link_libraries(test_state_machine ${LIBS} ${CMAKE_THREAD_LIBS_INIT})
#set_target_properties(test_state_machine PROPERTIES VERSION ${APP_VERSION})


#Documentation
#find_package(Doxygen REQUIRED)
#if(DOXYGEN_FOUND)
#    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
#    add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMENT "Generating API documentation with Doxygen" VERBATIM)
#endif(DOXYGEN_FOUND)
