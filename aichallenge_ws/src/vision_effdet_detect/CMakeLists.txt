cmake_minimum_required(VERSION 2.8.12)
project(vision_effdet_detect)

include(FindPkgConfig)

find_package(autoware_build_flags REQUIRED)
find_package(autoware_msgs REQUIRED)
find_package(autoware_config_msgs REQUIRED)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        tf
        autoware_msgs
        autoware_config_msgs
        )

find_package(catkin REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
add_definitions(-D WITH_CUDA)
add_definitions(-D__CUDA_NO_HALF_OPERATORS__)

catkin_package(
        CATKIN_DEPENDS cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        autoware_config_msgs
        tf
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
)


#####EFFDET########
##############################SSD'sFORK of CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
#set(EFFDET_CAFFE_PATH "$ENV{HOME}/ssdcaffe/distribute")
set(LIBTORCH_DIR $ENV{HOME}/aichallenge_ws/src/region_tlr_effnet/data/libtorch)
####################################################################################################

AW_CHECK_CUDA()

if (USE_CUDA)

    include_directories(${CUDA_INCLUDE_DIRS})

    IF ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^arm")
        link_directories(/usr/lib/arm-linux-gnueabihf/tegra)
    endif ()


    set(CMAKE_PREFIX_PATH $ENV{HOME}/aichallenge_ws/src/region_tlr_effnet/data/libtorch)

    set(CMAKE_INSTALL_RPATH "$ENV{HOME}/aichallenge_ws/src/region_tlr_effnet/data/libtorch/lib")
    set(CMAKE_INSTALL_RPATH "$ENV{HOME}/aichallenge_ws/src/vision_effdet_detect/data/nms/lib")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

    find_package(Torch REQUIRED)

    #CUDA_ADD_LIBRARY(
    #    CUDA_LIB
    #    include/nms/cpu/nms_cpu.cpp
    #    include/nms/cuda/nms_cuda.cu
    #)

    #target_link_libraries(CUDA_LIB 
    #    ${TORCH_LIBRARIES}
    #    ${PYTHON_LIBRARIES}
    #    ${CUDA_LIBRARIES}
    #)

    add_executable(vision_effDet_detect
            src/vision_effDet_detect.cpp
            src/vision_effDet_detect_node.cpp
            )

    target_link_libraries(vision_effDet_detect
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${OpenCV_LIBS}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${TORCH_LIBRARIES}
            $ENV{HOME}/aichallenge_ws/src/vision_effdet_detect/data/nms/lib/libCUDA_LIB.so
            #libCUDA_LIB.so
            #CUDA_LIB
            glog
            )

    #target_include_directories(vision_ssd_detect PRIVATE
    #        ${SSD_CAFFE_PATH}/include
    #        include
    #        )

    include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            ${PYTHON_INCLUDE_DIRS}
    )

    add_dependencies(vision_effDet_detect
            ${catkin_EXPORTED_TARGETS}
            )

    install(DIRECTORY include/
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

    install(TARGETS vision_effDet_detect
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            PATTERN ".svn" EXCLUDE)

    install(DIRECTORY data/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

else ()
    message("Unable to install because cuda is not used")
endif ()
