cmake_minimum_required( VERSION 2.8 )
project (practice)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3")

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
include_directories( ${PROJECT_SOURCE_DIR}/include )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# g2o Eigen Cholmod
find_package(G2O REQUIRED)
include_directories(
  ${G2O_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

find_package( Cholmod REQUIRED )
include_directories( ${CHOLMOD_INCLUDE_DIR} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
# difference between CERES_INCLUDE_DIRS and CERES_INCLUDE_DIR?

# gtsam
find_package( GTSAMCMakeTools )
find_package( GTSAM REQUIRED )
include_directories( ${GTSAM_INCLUDE_DIR} )
# gtsam


## executable
add_executable(mytest src/mytest.cpp)
target_link_libraries(mytest ${OpenCV_LIBS})


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

######### for BAL optimization with ceres

add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/src/BALProblem.cpp)
add_library(command_args SHARED ${PROJECT_SOURCE_DIR}/src/command_args.cpp)
add_executable(ceres_bal src/ceres_bal.cpp)
target_link_libraries(ceres_bal ${CERES_LIBRARIES} BALProblem command_args)

######### for BAL optimization with ceres


######### pose graph in g2o
add_executable(pose_graph_g2o_SE3 src/pose_graph_g2o_SE3.cpp)
target_link_libraries( pose_graph_g2o_SE3
    g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES}
)

add_executable(pose_graph_g2o_sophus src/pose_graph_g2o_sophus.cpp)
target_link_libraries(pose_graph_g2o_sophus 
    g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES} ${Sophus_LIBRARIES}
)
######### pose graph in g2o


######### pose graph in gtsam

add_executable(pose_graph_gtsam src/pose_graph_gtsam.cpp)
target_link_libraries( pose_graph_gtsam ${CHOLMOD_LIBRARIES} gtsam )

######### pose graph in gtsam


######### use_sophus
add_executable(use_sophus src/use_sophus.cpp)
target_link_libraries(use_sophus ${Sophus_LIBRARIES})

######### use_sophus
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

