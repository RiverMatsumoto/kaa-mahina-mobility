cmake_minimum_required(VERSION 3.8)
project(cr_mission_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-value)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(smacc2 REQUIRED)
find_package(nav2z_client REQUIRED)
find_package(geodesy REQUIRED)
find_package(cr_interfaces REQUIRED)
find_package(ros_timer_client REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(include
                    ${std_msgs_INCLUDE_DIRS}
                    ${smacc2_INCLUDE_DIRS}
                    ${nav2z_client_INCLUDE_DIRS}
                    ${cr_interfaces_INCLUDE_DIRS}
                    ${ros_timer_client_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_executable(${PROJECT_NAME}_node
                src/${PROJECT_NAME}/sm_trial_planner.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${smacc2_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_include_directories(
  ${PROJECT_NAME}_node PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${smacc2_INCLUDE_DIRS}
    ${nav2z_client_INCLUDE_DIRS}
    ${cr_interfaces_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_node smacc2 geodesy cr_interfaces std_srvs std_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
