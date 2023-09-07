cmake_minimum_required(VERSION 3.10)

# 项目名称
project(cam_tof_manual)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)

# 查找OpenCV库
find_package(OpenCV REQUIRED)

# 查找PCL库
find_package(PCL REQUIRED)

# 查找Eigen库
find_package(Eigen3 REQUIRED)

# 包含头文件目录
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)



# 添加PCL库的定义和编译选项
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# 指定源代码文件
# set(SOURCE_FILES src/main.cpp)
set(SOURCE_FILES src/main.cpp src/camera.cpp src/ir_camera.cpp src/optimizer.cpp src/optimizer_types.cpp) 

add_subdirectory(thirdparty/g2o)
# 创建可执行文件
add_executable(cam_tof_manual ${SOURCE_FILES})
# 链接库文件
target_link_libraries(cam_tof_manual ${OpenCV_LIBS})
target_link_libraries(cam_tof_manual ${PCL_LIBRARIES})
target_link_libraries(cam_tof_manual Eigen3::Eigen )
target_link_libraries(cam_tof_manual ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o.so)

# target_link_libraries(cam_tof_manual g2o)

# target_include_directories(cam_tof_manual PRIVATE ${PROJECT_SOURCE_DIR}/thirdparty/g2o)

