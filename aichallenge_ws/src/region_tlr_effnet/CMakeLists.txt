cmake_minimum_required(VERSION 2.8.3)
project(region_tlr_effnet)

find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(vector_map REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        cv_bridge
        geometry_msgs
        libvectormap
        roscpp
        roslint
        sensor_msgs
        std_msgs
        tf
        vector_map_server
        visualization_msgs
        autoware_msgs
        vector_map
        lanelet2_extension
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})

else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(OpenGL REQUIRED)

###################################
## catkin specific configuration ##
###################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
  autoware_msgs
  cmake_modules
  cv_bridge
  geometry_msgs
  libvectormap
  roscpp
  sensor_msgs
  std_msgs
  tf
  vector_map
  vector_map_server
  visualization_msgs

  LIBRARIES libcontext
)



set(CMAKE_CXX_FLAGS "-O2 -Wall -Wunused-variable ${CMAKE_CXX_FLAGS}")

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

###########
## Build ##
###########
add_library(libcontext lib/context.cpp)

add_dependencies(
  libcontext
  ${catkin_EXPORTED_TARGETS}
)

################################## region_tlr_effnet ##################################

##############################effnet'sFORK of CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
set(LIBTORCH_DIR $ENV{HOME}/aichallenge_ws/src/region_tlr_effnet/data/libtorch)
####################################################################################################

set(CMAKE_PREFIX_PATH $ENV{HOME}/aichallenge_ws/src/region_tlr_effnet/data/libtorch)
find_package(Torch REQUIRED)
find_package(CUDA REQUIRED)

set(CMAKE_INSTALL_RPATH "$ENV{HOME}/aichallenge_ws/src/region_tlr_effnet/data/libtorch/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(
  region_tlr_effnet
  nodes/region_tlr_effNet.cpp
  nodes/traffic_light_recognizer.cpp
)

target_link_libraries(
  region_tlr_effnet
  ${TORCH_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${CUDA_CUBLAS_LIBRARIES}
  ${CUDA_curand_LIBRARY}
  glog
  libcontext
)

target_include_directories(
  region_tlr_effnet PRIVATE
  ${CUDA_INCLUDE_DIRS}
)

add_dependencies(
  region_tlr_effnet
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS region_tlr_effnet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY data/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
