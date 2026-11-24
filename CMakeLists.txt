cmake_minimum_required(VERSION 3.8)
project(erl1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  urdf
  rviz
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS

  scripts/treasure_hunt.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
