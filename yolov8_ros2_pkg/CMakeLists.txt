cmake_minimum_required(VERSION 3.8)
project(yolov8_ros2_pkg)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(realsense2 REQUIRED)

# ONNX Runtime
set(ONNXRUNTIME_DIR "/home/m/Documents/onnxruntime-linux-x64-1.21.0")
message(STATUS "ONNXRUNTIME_DIR: ${ONNXRUNTIME_DIR}")

include_directories(
  include
  ${ONNXRUNTIME_DIR}/include
)

# === Target definition ===
add_executable(yolov8_ros2
  src/yolov8_ros2.cpp
  src/utils.cpp
  src/yolov8Predictor.cpp
)

# === Link Libraries ===
target_link_libraries(yolov8_ros2
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
  realsense2::realsense2
)

# === Ament Dependencies ===
ament_target_dependencies(yolov8_ros2
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

# === Install Target ===
install(TARGETS yolov8_ros2
  DESTINATION lib/${PROJECT_NAME})

# === Testing (Optional) ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === Platform-specific linking (optional, already done above) ===
# No need for these if already statically linked above
# But kept here for clarity/override
if(WIN32)
  target_link_libraries(yolov8_ros2 "${ONNXRUNTIME_DIR}/lib/onnxruntime.lib")
elseif(UNIX)
  target_link_libraries(yolov8_ros2 "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")
endif()

# === Mark as ament package ===
ament_package()

