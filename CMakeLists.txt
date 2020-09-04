cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(DeformableSLAM)

SET(CMAKE_BUILD_TYPE RELEASE)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -march=native -fopenmp -Wno-deprecated-declarations" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -fopenmp -Wno-deprecated-declarations")

# Check C++14 or C++0x support
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

set(CMAKE_PREFIX_PATH "/home/jose/pytorch/torch/share/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(Torch REQUIRED)
find_package(OpenCV 4.0.0 REQUIRED)
find_package(Pangolin REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

message("${TORCH_INCLUDE_DIRS}")
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/Thirdparty/ORBSLAM_2/include/
        ${PROJECT_SOURCE_DIR}/Thirdparty/BBS
        ${PROJECT_SOURCE_DIR}/Thirdparty/libelas-for-hamlyn/src
        ${PROJECT_SOURCE_DIR}/Dataloaders
        ${PROJECT_SOURCE_DIR}/Modules/Common
        ${PROJECT_SOURCE_DIR}/Modules/GroundTruth
        ${PROJECT_SOURCE_DIR}/Modules/Mapping
        ${PROJECT_SOURCE_DIR}/Modules/Matching
        ${PROJECT_SOURCE_DIR}/Modules/Segmentation
        ${PROJECT_SOURCE_DIR}/Modules/Settings
        ${PROJECT_SOURCE_DIR}/Modules/Template
        ${PROJECT_SOURCE_DIR}/Modules/Tracking
        ${PROJECT_SOURCE_DIR}/Modules/Viewer
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS} 
        ${CERES_INCLUDE_DIRS}
        ${TORCH_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
 
FILE(GLOB SRC_DEFSLAM_FILES
        "Modules/Common/*.cc"
        "Modules/GroundTruth/*.cc"
        "Modules/Mapping/*.cc"
        "Modules/Matching/*.cc"
        "Modules/Segmentation/*.cc"
        "Modules/Template/*.cc"
        "Modules/Tracking/*.cc"
        "Modules/Viewer/*.cc")

add_library(${PROJECT_NAME} SHARED
        ${SRC_DEFSLAM_FILES}
        )

add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/g2o)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/BBS)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/ORBSLAM_2)
add_subdirectory(${PROJECT_SOURCE_DIR}/Modules/ToolsPCL)
add_subdirectory(${PROJECT_SOURCE_DIR}/Dataloaders)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/libelas-for-hamlyn)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS} 
${Pangolin_LIBRARIES}
${CERES_LIBRARIES}
${TORCH_LIBRARIES}
DBoW2
g2o
BBS
ORBSLAM
ToolsPCL
libelas
)
 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Apps)

## Just receive image from a camera or a video and process them.
add_executable(DefSLAM Apps/simple_camera.cc)
target_link_libraries(DefSLAM ${PROJECT_NAME} ${TORCH_LIBRARIES})
set_property(TARGET DefSLAM PROPERTY CXX_STANDARD 14)

add_executable(DefSLAMGTCT Apps/simple_CT.cc)
target_link_libraries(DefSLAMGTCT ${PROJECT_NAME} ${TORCH_LIBRARIES}) 
set_property(TARGET DefSLAMGTCT PROPERTY CXX_STANDARD 14)

add_executable(DefSLAMGT Apps/stereo_groundtruth.cc)
target_link_libraries(DefSLAMGT ${PROJECT_NAME} ${TORCH_LIBRARIES})
set_property(TARGET DefSLAMGT PROPERTY CXX_STANDARD 14)

add_executable(DefSLAMHamyln Apps/stereo_Hamlyn.cc)
target_link_libraries(DefSLAMHamyln ${PROJECT_NAME} Dataloader ${TORCH_LIBRARIES})
set_property(TARGET DefSLAMHamyln PROPERTY CXX_STANDARD 14)
