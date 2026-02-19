cmake_minimum_required(VERSION 3.8)
project(ros2_fundamentals_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Sets the currently required dependencies so that build is more simple
set(dependencies
  rclcpp
  std_msgs
)

# Specifies directories to the list of paths that the compiler uses to search for header files.
# Important for C++ projects when you have custom header files which aren't located in the standard
# system include paths.
include_directories(
  include
)

# Tell CMake to create an executable target
#add_executable(minimal_cpp_publisher src/minimal_cpp_publisher.cpp)
#ament_target_dependencies(minimal_cpp_publisher ${dependencies}) # Tells compiler the dependencies needed for the executable

#add_executable(minimal_cpp_subscriber src/minimal_cpp_subscriber.cpp)
#ament_target_dependencies(minimal_cpp_subscriber ${dependencies})

# Create designated location where we are going to copy necessary files 
install (
  DIRECTORY ros2_fundamentals_examples # files for development
  DESTINATION share/${PROJECT_NAME} # files for actual build
)

# Special area for includes / header files
install (
  DIRECTORY include/
  DESTINATION include
)

# Install C++ executables
#install (
# TARGETS
# minimal_cpp_publisher
# minimal_cpp_subscriber
# DESTINATION lib/${PROJECT_NAME}
#)

# Install Python modules for import
ament_python_install_package(${PROJECT_NAME})

# Add this section to install Python scripts
install (
  PROGRAMS
  ros2_fundamentals_examples/py_minimal_publisher.py
  DESTINATION lib/${PROJECT_NAME}
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

# Exports include directories of a package so the can be easily included into other packages which depend on it
ament_export_include_directories(include)

ament_package()
