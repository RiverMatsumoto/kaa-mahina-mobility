cmake_minimum_required(VERSION 3.10)
# set the project name
project(libroboclaw VERSION 0.1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

configure_file(libroboclawConfig.h.in libroboclawConfig.h)

find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)
find_package(PkgConfig REQUIRED)
# Find libudev
pkg_check_modules(UDEV REQUIRED libudev)

############################################################
# Create a library
############################################################

# add library
add_library(libroboclaw
    src/roboclaw_driver.cpp
    src/TimeoutSerial.cpp
)
target_include_directories(libroboclaw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(libroboclaw gtest pthread ${Boost_LIBRARIES} ${UDEV_LIBRARIES})

############################################################
# Create an executable
############################################################
add_executable(test_roboclaw_connection 
    examples/test_roboclaw_connection.cpp)
target_include_directories(test_roboclaw_connection PUBLIC "${PROJECT_BINARY_DIR}")
target_link_libraries(test_roboclaw_connection PRIVATE libroboclaw)

# install library
install(TARGETS libroboclaw
    EXPORT export_libroboclaw
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
# install executable
install(TARGETS test_roboclaw_connection
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ 
    DESTINATION include
)

# export targets for libroboclaw
export(TARGETS libroboclaw FILE libroboclawTargets.cmake)
