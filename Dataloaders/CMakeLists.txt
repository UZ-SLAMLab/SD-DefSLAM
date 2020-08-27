cmake_minimum_required(VERSION 2.8)
project(Dataloader)

find_package(Eigen3 REQUIRED)

find_package(OpenCV 4.2.0 REQUIRED)
find_package(Boost 1.60 REQUIRED COMPONENTS filesystem)

include_directories(
    ${PROJECT_SOURCE_DIR} 
    ${OpenCV_INCLUDE_DIRS} 
    ${Boost_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
     StereoRectifier.cc
     Hamlyn/dataloader.cc
     )

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Boost::filesystem)
