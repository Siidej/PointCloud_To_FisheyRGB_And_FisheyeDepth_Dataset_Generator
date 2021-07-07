cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 20)

project(world2cam)
set(SOURCES
    src/main.cpp
    src/PTXReader.cpp
    src/CameraLoader.cpp
    src/utils.cpp
    )
set(HEADERS
    include/PTXReader.hpp
    include/CameraLoader.hpp
    include/utils.hpp
    )

find_package( OpenCV REQUIRED )
find_package( Eigen3 3.3 REQUIRED NO_MODULE )

include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( world2cam ${SOURCES} )
target_link_libraries( world2cam ${OpenCV_LIBS} )
target_link_libraries( world2cam Eigen3::Eigen)

target_include_directories(world2cam
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)

