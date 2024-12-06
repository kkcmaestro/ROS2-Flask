cmake_minimum_required(VERSION 3.8)
project(Ubox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
include_directories(include)
# find_package(<dependency> REQUIRED)

add_executable(cpp_exe src/ScriptARos.cpp)
ament_target_dependencies(cpp_exe rclcpp geometry_msgs)

install(TARGETS 
 cpp_exe
  DESTINATION lib/${PROJECT_NAME}
  )


add_executable(velocity_publisher src/velocity_publisher.cpp)
ament_target_dependencies(velocity_publisher rclcpp geometry_msgs)

install(TARGETS
  velocity_publisher
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
