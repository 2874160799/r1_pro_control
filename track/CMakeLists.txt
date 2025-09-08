cmake_minimum_required(VERSION 3.8)
project(track)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)

# 先声明接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SkeletonPoints.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 让你的节点看到生成的消息头文件
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_executable(track_node src/track.cpp)

ament_target_dependencies(track_node
  rclcpp
  std_msgs 
  geometry_msgs
)

# 这行非常重要，确保在生成接口后再链接类型支持
rosidl_target_interfaces(track_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(track_node ${OpenCV_LIBS})

install(TARGETS track_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch msg
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
