#This is the name of the current project. Therefore the name of PROJECT_BINARY_DIR is now ISM_BINARY_DIR.
project(ISM)
cmake_minimum_required(VERSION 2.4)

# validation
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -std=gnu++0x -O2")

# GCC only sends a warning and not an error if it can't compile with stack canaries.
set(CMAKE_REQUIRED_FLAGS "-Werror")

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_subdirectory(ISM ./build)
GET_DIRECTORY_PROPERTY(ISM_INCLUDE_DIR DIRECTORY ISM DEFINITION ISM_INCLUDE_DIR)
GET_DIRECTORY_PROPERTY(ISM_LIBRARY_DIR DIRECTORY ISM DEFINITION ISM_LIBRARY_DIR)
set(ISM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} ${ISM_INCLUDE_DIR})
include_directories(${ISM_INCLUDE_DIR})
link_directories(${ISM_LIBRARY_DIR})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${ISM_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ISM_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${ISM_BINARY_DIR}/bin)


add_executable(evaluation-trainer src/trainer.cpp)
target_link_libraries(evaluation-trainer ism ${Boost_LIBRARIES})
add_executable(validator src/validator.cpp)
target_link_libraries(validator ism ${Boost_LIBRARIES})
add_executable(dataMerger src/dataMerger.cpp)
target_link_libraries(dataMerger ism ${Boost_LIBRARIES})
add_executable(modelCleaner src/modelCleaner.cpp)
target_link_libraries(modelCleaner ism ${Boost_LIBRARIES})
add_executable(recordCleaner src/recordCleaner.cpp)
target_link_libraries(recordCleaner ism ${Boost_LIBRARIES})
add_executable(testRunner src/testRunner.cpp)
target_link_libraries(testRunner ism ${Boost_LIBRARIES})
add_executable(markerRotator src/markerRotator.cpp)
target_link_libraries(markerRotator ism ${Boost_LIBRARIES})
add_executable(poseInterpolator src/poseInterpolator.cpp)
target_link_libraries(poseInterpolator ism ${Boost_LIBRARIES})
add_executable(rotationInvariantRotator src/rotationInvariantObjectsRotator.cpp)
target_link_libraries(rotationInvariantRotator ism ${Boost_LIBRARIES})
add_executable(recordedObjectsTransformer src/recordedObjectsTransformer.cpp)
target_link_libraries(recordedObjectsTransformer ism ${Boost_LIBRARIES})

#Code for evaluation
#add_library(evaluation src/test_cases/randomDemoRecorder.cpp)
#add_executable(simpleEvaluator src/test_cases/simpleEvaluator.cpp)
#target_link_libraries(simpleEvaluator ism ${Boost_LIBRARIES} evaluation)
#add_executable(randomTestRunner src/test_cases/randomTestRunner.cpp src/test_cases/randomDemoRecorder.cpp)
#target_link_libraries(randomTestRunner ism ${Boost_LIBRARIES})
