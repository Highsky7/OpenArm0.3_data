cmake_minimum_required(VERSION 3.8)
project(openarm_static_bimanual_bringup)

# 컴파일러 경고 옵션(관성)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수
find_package(ament_cmake REQUIRED)

# --- 테스트(선택) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스/포맷 검사 스킵 옵션은 필요시 조정
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ========== 설치 규칙 ==========
# 1) launch / config 디렉터리
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 2) RViz 설정이 별도 디렉터리에 있다면(선택)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 3) 실행 가능한 파이썬 스크립트들
#    - 각 파일에 반드시 shebang: #!/usr/bin/env python3
#    - chmod +x scripts/*.py 해두면 좋음(깃에 권한 포함 권장)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  install(PROGRAMS
    scripts/jsp_fpc_bridge.py
    scripts/test_trajectory_node.py
    scripts/gravity_comp_node.py
    scripts/continuous_recorder_node.py
    scripts/keyboard_gripper_controller.py
    scripts/fmvla_data_record.py
    scripts/trajectory_replay_node.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
