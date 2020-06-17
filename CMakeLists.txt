cmake_minimum_required(VERSION 2.8)
project(DeformableSLAM)

SET(CMAKE_BUILD_TYPE DEBUG)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -march=native -fopenmp -Wno-deprecated-declarations" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -fopenmp -Wno-deprecated-declarations")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC14)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


find_package(OpenCV 4.0.0 REQUIRED)
find_package(Pangolin REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(VTK REQUIRED)
find_package(Eigen3 REQUIRED)

message(STATUS "Using VTK ${VTK_VERSION}")

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/Thirdparty/TPS
${PROJECT_SOURCE_DIR}/Thirdparty/timer
${PROJECT_SOURCE_DIR}/Thirdparty/Poisson
)

INCLUDE(${VTK_USE_FILE})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

FILE(GLOB SRC_FOLDER_FILES
     "src/*.cc")

add_library(${PROJECT_NAME} SHARED
${SRC_FOLDER_FILES}
Thirdparty/timer/Timer.cpp
)

add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/g2o)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/Poisson)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${CERES_LIBRARIES}
${VTK_LIBRARIES}
DBoW2
g2o
POISSON
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(simple
Examples/Monocular/simple.cc)
target_link_libraries(simple ${PROJECT_NAME})

add_executable(simpleCT
Examples/Monocular/simple-CT.cc)
target_link_libraries(simpleCT ${PROJECT_NAME})

add_executable(simplestereo
Examples/Monocular/stereo_groundtruth.cc)
target_link_libraries(simplestereo ${PROJECT_NAME})

add_executable(Mandala
Examples/Monocular/Mandala.cc)
target_link_libraries(Mandala ${PROJECT_NAME})