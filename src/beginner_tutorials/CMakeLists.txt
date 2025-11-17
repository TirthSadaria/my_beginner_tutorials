cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Create library with publisher implementation
add_library(minimal_publisher_lib src/minimal_publisher.cpp)
target_include_directories(minimal_publisher_lib PUBLIC include)
ament_target_dependencies(minimal_publisher_lib
  rclcpp
  std_msgs
  rosidl_default_runtime
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)
target_link_libraries(minimal_publisher_lib "${cpp_typesupport_target}")

# Create talker executable
add_executable(talker src/publisher_member_function.cpp)
target_link_libraries(talker minimal_publisher_lib)
# Set RPATH so executable can find the library at runtime
# This ensures the library is found without needing LD_LIBRARY_PATH
set_target_properties(talker PROPERTIES
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}"
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Create listener executable
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener
  rclcpp
  std_msgs
)

# Install executables and library
install(TARGETS
  talker
  listener
  minimal_publisher_lib
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS scripts/generate_tf_frames.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(Catch2 2 REQUIRED)
  set(TEST_SOURCES
    test/talker_integration_test.cpp
  )

  add_executable(talker_integration_test ${TEST_SOURCES})
  target_link_libraries(talker_integration_test
    "${cpp_typesupport_target}"
    minimal_publisher_lib
    Catch2::Catch2
  )
  ament_target_dependencies(talker_integration_test
    rclcpp
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  add_test(NAME talker_integration_test COMMAND $<TARGET_FILE:talker_integration_test>)
  set_tests_properties(talker_integration_test PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test/ament_cmake_test_stub:$ENV{PYTHONPATH}")
endif()

target_link_libraries(talker "${cpp_typesupport_target}")

# Export library so it can be found at runtime
# The library path will be added to LD_LIBRARY_PATH by the setup script
ament_export_libraries(minimal_publisher_lib)

ament_package()
