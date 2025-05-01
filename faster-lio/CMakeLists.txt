cmake_minimum_required(VERSION 3.5)
project(faster_lio)

## 查找必要依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## 添加消息定义
add_message_files(
  FILES
  Pose6D.msg
)

## 生成消息头文件
generate_messages(
  DEPENDENCIES
  std_msgs
)

## 查找其他依赖
find_package(PCL REQUIRED COMPONENTS filters segmentation)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)

## catkin 包配置
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

## 编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

## 设置 ivox node 类型
option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)
if (WITH_IVOX_NODE_TYPE_PHC)
    message("USING_IVOX_NODE_TYPE_PHC")
    add_definitions(-DIVOX_NODE_TYPE_PHC)
else ()
    message("USING_IVOX_NODE_TYPE_DEFAULT")
endif()

## include 路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src/graph_optimization
  /home/ouymq/catkin_ws/src/Sophus          # <<=== 添加这一行用于包含 sophus/so3.hpp
)

## link 路径
link_directories(${PCL_LIBRARY_DIRS})

## 添加子目录
add_subdirectory(src)
add_subdirectory(app)  # 可选，如果有 app 目录

## 如果需要手动依赖目标（确保生成自定义消息）
# add_dependencies(your_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

