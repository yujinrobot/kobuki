cmake_minimum_required(VERSION 3.5)
project(kobuki_velocity_smoother)

find_package(ament_cmake_ros REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_threads REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/velocity_smoother.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "ecl_threads"
  "geometry_msgs"
  "nav_msgs"
  "rcl_interfaces"
  "rclcpp"
  "rclcpp_components"
)
target_link_libraries(${PROJECT_NAME}
  ecl_threads::ecl_threads
)

add_executable(kobuki_velocity_smoother_node src/velocity_smoother_node.cpp)
ament_target_dependencies(kobuki_velocity_smoother_node
  "rclcpp"
)
target_link_libraries(kobuki_velocity_smoother_node
  ${PROJECT_NAME}
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "kobuki_velocity_smoother::VelocitySmoother")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS kobuki_velocity_smoother_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
