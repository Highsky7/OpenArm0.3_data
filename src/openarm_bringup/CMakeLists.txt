# Copyright 2025 Reazon Holdings, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(openarm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# launch 폴더만 확실히 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# config 폴더가 있을 때만 설치
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 실행 가능한 파이썬 스크립트를 lib/<pkg> 아래에 설치 → Node(executable=...)로 바로 실행 가능
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  # 필요 스크립트만 명시해도 되고, 아래처럼 *.py 전체를 잡아도 됨
  # 권장: 명시형 (지금은 퍼블리셔만)
  install(PROGRAMS
    scripts/teleop_serial_publisher.py
    scripts/teleop_follower.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
