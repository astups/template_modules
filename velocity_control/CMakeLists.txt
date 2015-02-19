cmake_minimum_required(VERSION 2.8)
project(velocity_control CXX C)

set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "5")
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





#Add header files
include_directories(${PROJECT_SOURCE_DIR})

#Add a library
#add_library(<lib> SHARED <source>)
#target_link_libraries(<lib> ${LIBS})
#set(LIBS <lib> ${LIBS})
#set_target_properties(<lib> PROPERTIES VERSION ${APP_VERSION}) #To apply the version number

#Add an executable
add_executable(main main.cpp)
target_link_libraries(main ${LIBS} ${CMAKE_THREAD_LIBS_INIT})
set_target_properties(main PROPERTIES VERSION ${APP_VERSION})





#Documentation
#find_package(Doxygen REQUIRED)
#if(DOXYGEN_FOUND)
#    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
#    add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMENT "Generating API documentation with Doxygen" VERBATIM)
#endif(DOXYGEN_FOUND)
