cmake_minimum_required(VERSION 2.8.3)
project(asr_lib_ism)

# We need c++11 support, because some functions are written in c++11
#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
add_definitions(-Wall -pedantic -std=gnu++0x -O2)

find_package(catkin REQUIRED)


###################################
# Build libism together with its db interface soci.
############################################################

#Tell cmake where libism resides, set lib and inc paths for example apps to be able to use it.
add_subdirectory(libism)
#Get directory paths.
GET_DIRECTORY_PROPERTY(ISM_INCLUDE_DIR DIRECTORY libism DEFINITION ISM_INCLUDE_DIR)
GET_DIRECTORY_PROPERTY(ISM_LIBRARY_DIR DIRECTORY libism DEFINITION ISM_LIBRARY_DIR)

#Add include directories to the build.
include_directories(
	${ISM_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)

# Make sure the linker can find the ism library once it is built.
link_directories(${ISM_LIBRARY_DIR})

# Added libism as include_dir to provide the Recognizer.hpp to another package.
catkin_package(
    INCLUDE_DIRS ${ISM_INCLUDE_DIR} libism
    LIBRARIES ism
)