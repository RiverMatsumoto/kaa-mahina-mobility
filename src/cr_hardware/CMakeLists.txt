cmake_minimum_required(VERSION 3.16)
project(cr_hardware LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wno-unused-function)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  # start ros2 control required
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  # end ros2 control required 
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# compile custom ros2 control SystemInterface plugin
add_library(
  cr_hardware
  SHARED
  src/differential_drive.cpp
  src/roboclaw.cpp
)
target_compile_features(cr_hardware PUBLIC cxx_std_17)
target_include_directories(cr_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cr_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(test_roboclaw 
  src/test_roboclaw.cpp 
  src/roboclaw.cpp)
target_include_directories(test_roboclaw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(test_roboclaw PUBLIC cr_hardware)

pluginlib_export_plugin_description_file(hardware_interface cr_hardware.xml)

install(TARGETS
  cr_hardware
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include/cr_hardware
)
# script to test roboclaw connections and commands
install(TARGETS test_roboclaw
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY description/ros2_control description/urdf config launch
  DESTINATION share/cr_hardware
)
install(TARGETS cr_hardware
  EXPORT export_cr_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

## EXPORTS
ament_export_targets(export_cr_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
