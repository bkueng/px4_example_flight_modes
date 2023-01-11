cmake_minimum_required(VERSION 3.5)
project(flight_mode_rtl)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_sdk_lib REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})
add_executable(flight_mode_rtl
	src/flight_mode_rtl.cpp)
ament_target_dependencies(flight_mode_rtl Eigen3 px4_sdk_lib rclcpp)

install(TARGETS
	flight_mode_rtl
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
