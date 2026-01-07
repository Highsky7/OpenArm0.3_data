cmake_minimum_required(VERSION 3.8)
project(openarm_static_bimanual_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# 빌드시 xacro 실행은 안하지만, 개발 편의를 위해 찾아둬도 무방
# find_package(xacro REQUIRED)  # 필요 없으면 주석 유지

# 리소스 설치 (URDF/Xacro, meshes, launch, rviz)
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 패키지 export
ament_package()
