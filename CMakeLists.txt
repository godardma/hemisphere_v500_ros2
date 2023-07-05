cmake_minimum_required(VERSION 3.10)
project(hemisphere_v500)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(NOT TARGET munu_io)
    find_package(munu_io REQUIRED)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/StampedString.msg"
    "msg/Binary1.msg"
    "msg/Binary2.msg"
    "msg/Binary3.msg"
    DEPENDENCIES std_msgs
)

include_directories(include ${Boost_INCLUDE_DIRS})
add_executable(hemisphere_v500_node src/hemisphere_v500_node.cpp)
target_compile_features(hemisphere_v500_node PUBLIC cxx_std_17)
ament_target_dependencies(hemisphere_v500_node rclcpp std_msgs munu_io)

target_include_directories(hemisphere_v500_node PUBLIC
    include ${Boost_INCLUDE_DIRS}
)
target_include_directories(hemisphere_v500_node PRIVATE src)


ament_target_dependencies(hemisphere_v500_node 
  rclcpp
  std_msgs
)

target_link_libraries(hemisphere_v500_node
    munu_io
)

rosidl_target_interfaces(hemisphere_v500_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  hemisphere_v500_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
