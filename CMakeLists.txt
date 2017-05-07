cmake_minimum_required( VERSION 2.8 )
project (practice)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3")

# list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
# difference between CERES_INCLUDE_DIRS and CERES_INCLUDE_DIR?

include_directories( ${PROJECT_SOURCE_DIR}/include )

add_executable(image_pyramid src/image_pyramid.cpp)
target_link_libraries(image_pyramid ${OpenCV_LIBS})

add_executable(sift src/sift.cpp)
target_link_libraries(sift ${OpenCV_LIBS})

add_executable(goodFeatureToTrack src/goodFeatureToTrack.cpp)
target_link_libraries(goodFeatureToTrack ${OpenCV_LIBS})

add_executable(blur src/blur.cpp)
target_link_libraries(blur ${OpenCV_LIBS})

add_executable(ceres_fitting src/ceres_fitting.cpp)
target_link_libraries(ceres_fitting ${OpenCV_LIBS} ${CERES_LIBRARIES})

