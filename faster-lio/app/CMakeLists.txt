include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/../Sophus
    /usr/include/suitesparse
)

add_executable(run_mapping_online run_mapping_online.cc)
target_link_libraries(run_mapping_online
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}    # ✅ 这行包含 tf, rosbag 等所有 ROS 库
    ${PCL_LIBRARIES}
    gflags
)

add_executable(run_mapping_offline run_mapping_offline.cc)
target_link_libraries(run_mapping_offline
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}    # ✅ 同样，间接链接 rosbag 等
    ${PCL_LIBRARIES}
    gflags
)

