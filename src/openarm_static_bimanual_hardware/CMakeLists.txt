cmake_minimum_required(VERSION 3.8)
project(openarm_static_bimanual_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/openarm_hw_flex.cpp
  src/canbus.cpp
  src/motor.cpp
  src/motor_control.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rcutils
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE OPENARM_STATIC_BIMANUAL_HARDWARE_BUILDING_DLL
)

pluginlib_export_plugin_description_file(hardware_interface
  openarm_static_bimanual_hardware_plugins.xml
)

# ---- Export 들 (외부 패키지가 사용할 수 있게) ----
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp rcutils)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# ---- 설치 (EXPORT 추가!) ----
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}                # ★ 이것이 있어야 위 ament_export_targets와 짝이 맞음
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES openarm_static_bimanual_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
