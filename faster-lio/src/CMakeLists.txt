add_library(${PROJECT_NAME}
    laser_mapping.cc
    pointcloud_preprocess.cc
    options.cc
    utils.cc
     cloud_preprocess.cpp    # ← 新增的cloud_filter.cc，必须加在这里
)

# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp livox_ros_driver_gencpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PYTHON_LIBRARIES}
    /usr/local/lib/libtbb.so
    /usr/local/lib/libtbbmalloc.so
    /usr/local/lib/libtbbmalloc_proxy.so
    tbb
    glog
    yaml-cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})

