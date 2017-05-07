cmake_minimum_required( VERSION 2.8 )
project (practice)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3")

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# g2o Eigen
find_package(G2O REQUIRED)
include_directories(
  ${G2O_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
# difference between CERES_INCLUDE_DIRS and CERES_INCLUDE_DIR?

include_directories( ${PROJECT_SOURCE_DIR}/include )

## executable

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

add_executable(robust_ceres_fitting src/robust_ceres_fitting.cpp)
target_link_libraries(robust_ceres_fitting ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(ceres_bundle_adjustment src/ceres_bundle_adjustment.cpp)
target_link_libraries(ceres_bundle_adjustment ${CERES_LIBRARIES})

add_executable(ceres_helloworld src/ceres_helloworld.cpp)
target_link_libraries(ceres_helloworld ${CERES_LIBRARIES})

if (GFLAGS)
  # The CERES_GFLAGS_NAMESPACE compile definition is NOT stored in
  # CERES_COMPILE_OPTIONS (and thus config.h) as Ceres itself does not
  # require gflags, only the tests and examples do.
  add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

  add_executable(ceres_powell src/ceres_powell.cpp)
  target_link_libraries(ceres_powell ${CERES_LIBRARIES})

endif (GFLAGS)

add_executable(g2o_fitting src/g2o_fitting.cpp)
target_link_libraries(g2o_fitting 
  ${OpenCV_LIBS}
  g2o_core g2o_stuff
)

