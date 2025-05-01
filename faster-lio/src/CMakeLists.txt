find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    tf
    rosbag
)

find_package(PCL REQUIRED COMPONENTS filters segmentation io)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)

# 添加 g2o 和 SuiteSparse 的头文件路径
include_directories(${G2O_INCLUDE_DIRS})
include_directories(/usr/include/suitesparse)

# 添加项目 include、catkin、PCL、Sophus（根目录，不带 /sophus）
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/../Sophus
)

# 添加主库
add_library(${PROJECT_NAME}
    laser_mapping.cc
    pointcloud_preprocess.cc
    options.cc
    utils.cc
    cloud_preprocess.cpp
    graph_optimization/graph_optimizer.cpp
    graph_optimization/loop_detector.cpp
)

# 添加依赖消息生成
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接库
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    glog
    yaml-cpp
    tbb
    gflags
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    g2o_solver_csparse
    g2o_csparse_extension
    g2o_types_slam3d       # ✅ 添加 SLAM3D 类型定义
)

# Python 头文件路径（如有）
target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})

