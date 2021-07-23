
# python bindings
find_package(PythonLibs 3.6 REQUIRED)
find_package(Boost COMPONENTS python3 REQUIRED)
add_library(python_pose_ekf MODULE  SHARED python_pose_ekf.cpp)
include_directories(../include
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIR}
        )
target_link_libraries(python_pose_ekf
        ${Boost_LIBRARIES}
        ${PYTHON_LIBRARIES}
        ${PROJECT_NAME} # pose_ekf library
        )
target_include_directories(python_pose_ekf PRIVATE ${PYTHON_INCLUDE_DIRS})

# Copy the __init__.py file
configure_file(__init__.py ${CMAKE_CURRENT_BINARY_DIR}/src/__init__.py COPYONLY)

# Suppress prefix "lib" because Python does not allow this prefix
set_target_properties(python_pose_ekf PROPERTIES PREFIX "")
