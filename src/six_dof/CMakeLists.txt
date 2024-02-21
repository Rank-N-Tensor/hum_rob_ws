cmake_minimum_required(VERSION 3.8)
project(six_dof)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pinocchio REQUIRED)

message(STATUS "Pinocchio found at: ${PINOCCHIO_INCLUDE_DIRS}")


find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_executable(test_node src/IK_test.cpp)
ament_target_dependencies(test_node rclcpp)
target_link_libraries(test_node pinocchio::pinocchio) #modern cmake namespace call

# target_include_directories(test_node PRIVATE
#   ${EIGEN3_INCLUDE_DIR}
#   ${PINOCCHIO_INCLUDE_DIRS}
# )




install(
  DIRECTORY
    urdf
    rviz
    launch
    meshes
  DESTINATION
    share/${PROJECT_NAME}/
)

install(TARGETS 
test_node
DESTINATION
lib/${PROJECT_NAME}/
)

ament_package()
