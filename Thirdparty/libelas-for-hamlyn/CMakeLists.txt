# project
cmake_minimum_required (VERSION 3.9)
project (libelas)

# directories
set (LIBELAS_SRC_DIR src)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
 

# include directory
include_directories(${LIBELAS_SRC_DIR}
                     ${PROJECT_SOURCE_DIR}/src)

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

# sources
FILE(GLOB LIBELAS_SRC_FILES "src/*.cpp")
 
# Tools dataset   
set(DATASET_TOOLS 
    "dataset/stereorectifier.h" 
    "dataset/dataloader.h"
    )

add_library(${PROJECT_NAME} SHARED
   ${LIBELAS_SRC_FILES}
)

include_directories(
    ${PROJECT_SOURCE_DIR}/dataset
)
# make release version
set(CMAKE_BUILD_TYPE RELEASE)
 
find_package(OpenCV 4.2.0 REQUIRED)
find_package(Boost 1.60 REQUIRED COMPONENTS filesystem)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
# build demo program
add_executable(elas main.cpp )

target_link_libraries(elas ${LIBELAS_SRC_FILES} ${OpenCV_LIBS})
 
add_executable(elasHamlyn mainHamlyn.cpp  ${DATASET_TOOLS})
 
target_link_libraries(elasHamlyn ${LIBELAS_SRC_FILES} ${OpenCV_LIBS} Boost::filesystem)

add_executable(rectifier generateRectImages.cpp ${DATASET_TOOLS})
  
target_link_libraries(rectifier ${LIBELAS_SRC_FILES} ${OpenCV_LIBS} Boost::filesystem)