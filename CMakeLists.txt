cmake_minimum_required(VERSION 3.5)
project(kansei)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jitsuyo REQUIRED)
find_package(kansei_interfaces REQUIRED)
find_package(keisan REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tachimawari REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/fallen/node/fallen_determinant.cpp"
  "src/${PROJECT_NAME}/fallen/node/fallen_node.cpp"
  "src/${PROJECT_NAME}/measurement/node/measurement_unit.cpp"
  "src/${PROJECT_NAME}/measurement/node/measurement_node.cpp"
  "src/${PROJECT_NAME}/measurement/filter/filter.cpp"
  "src/${PROJECT_NAME}/measurement/filter/madgwick/madgwick_filter.cpp"
  "src/${PROJECT_NAME}/measurement/filter/madgwick/stateless_orientation.cpp"
  "src/${PROJECT_NAME}/measurement/sensor/mpu.cpp"
  "src/${PROJECT_NAME}/node/kansei_node.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  jitsuyo
  keisan
  kansei_interfaces
  rclcpp
  std_msgs
  tachimawari
  tf2
  tf2_geometry_msgs)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(check_rpy "src/check_rpy_main.cpp")
target_include_directories(check_rpy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(check_rpy ${PROJECT_NAME})

add_executable(main "src/kansei_main.cpp")
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(main ${PROJECT_NAME})

install(TARGETS
  check_rpy
  main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_tests
    "test/fallen/determinant_test.cpp"
    "test/fallen/fallen_test.cpp"
    "test/measurement/measurement_test.cpp"
    "test/node/kansei_node_test.cpp")

  target_include_directories(${PROJECT_NAME}_tests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  geometry_msgs
  jitsuyo
  keisan
  kansei_interfaces
  rclcpp
  std_msgs
  tachimawari
  tf2
  tf2_geometry_msgs)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
