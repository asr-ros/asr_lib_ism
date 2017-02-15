project(LIBISM)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/soci/src/cmake/modules/")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#SOCI is a database access library for C++ that makes the illusion of embedding SQL queries in the regular C++ code, staying entirely within the Standard C++.
set(SOCI_TESTS OFF CACHE BOOL "")

set(WITH_DB2 OFF CACHE BOOL "")
set(SOCI_DB2 OFF CACHE BOOL "")

set(WITH_FIREBIRD OFF CACHE BOOL "")
set(SOCI_FIREBIRD OFF CACHE BOOL "")

set(WITH_MYSQL OFF CACHE BOOL "")
set(SOCI_MYSQL OFF CACHE BOOL "")

set(WITH_ODBC OFF CACHE BOOL "")
set(SOCI_ODBC OFF CACHE BOOL "")

set(WITH_ORACLE OFF CACHE BOOL "")
set(SOCI_ORACLE OFF CACHE BOOL "")

set(WITH_POSTGRESQL OFF CACHE BOOL "")
set(SOCI_POSTGRESQL OFF CACHE BOOL "")


add_subdirectory(soci/src ./build)

find_package(Boost COMPONENTS program_options thread REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.7 REQUIRED COMPONENTS common io filters segmentation)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PCL_COMMON_INCLUDE_DIR})
include_directories(${PCL_IO_INCLUDE_DIR})
include_directories(${PCL_SEGMENTATION_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(${CMAKE_VERSION})

# validation
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-maybe-uninitialized -Wextra -pedantic -std=gnu++0x -O2")
# GCC only sends a warning and not an error if it can't compile with stack canaries.
set(CMAKE_REQUIRED_FLAGS "-Werror")

FILE(GLOB CPPFILES *.cpp
		   common_type/*.cpp
		   heuristic_trainer/*.cpp
		   combinatorial_trainer/*.cpp
		   combinatorial_optimization/*.cpp
		   recognizer/*.cpp
		   recorder/*.cpp
		   utility/*.cpp
		   tools/*.cpp)

set(ISM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/soci/src/core ${CMAKE_CURRENT_SOURCE_DIR}/soci/src/backends/sqlite3 ${EIGEN3_INCLUDE_DIR} ${BOOST_INCLUDE_DIR})
set(ISM_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR})


# Changed the library output directory to be able to use the library with catkin
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBISM_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBISM_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LIBISM_BINARY_DIR}/bin)

include_directories(${ISM_INCLUDE_DIR})
add_library(ism SHARED ${CPPFILES})
target_link_libraries(ism ${LIBS} ${Boost_LIBRARIES} soci_core soci_sqlite3 pcl_common pcl_filters pcl_io pcl_segmentation)
set_target_properties(ism PROPERTIES LINKER_LANGUAGE CXX)

