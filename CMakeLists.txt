cmake_minimum_required(VERSION 3.5)
project(pose_ekf)

set(CMAKE_CXX_STANDARD 11)


include_directories(include)

# make library
set(KF_SOURCE_FILES
        KF/ekf.cpp
        KF/ukf.cpp
        KF/filter_base.cpp
        KF/filter_utilities.cpp
        KF/ekf_interface.cpp )

set(KF_HEADER_FILES
        include/robot_localization/ekf.h
        include/robot_localization/filter_base.h
        include/robot_localization/filter_common.h
        include/robot_localization/filter_utilities.h
        include/robot_localization/ukf.h
        include/robot_localization/ekf_interface.h
        )

find_package (Eigen3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME} SHARED ${KF_SOURCE_FILES} ${KF_HEADER_FILES})
include_directories(${EIGEN3_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}  ${EIGEN3_LIBRARIES})

add_subdirectory(python_bindings)
