cmake_minimum_required(VERSION 3.8)
project(rmd_x8_90_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(socketcan_adapter REQUIRED)
# Dependencies needed for socketcan adapter
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(can_msgs REQUIRED)

set(dependencies
  socketcan_adapter
  # Dependencies needed for socketcan adapter
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  can_msgs
)

set(library_name ${PROJECT_NAME})

add_library(
  ${library_name} SHARED
  src/rmd_x8_90_driver.cpp
)

target_compile_features(${library_name} PUBLIC cxx_std_17)

target_include_directories(
  ${library_name} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${library_name}>
)

ament_target_dependencies(${library_name} PUBLIC
  ${dependencies}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "rmd_x8_90_driver_BUILDING_DLL")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(Catch2 2 REQUIRED)
  add_executable(rmd_x8_90_driver_test test/rmd_x8_90_driver_test.cpp)
  target_link_libraries(rmd_x8_90_driver_test Catch2::Catch2 ${library_name})
  ament_target_dependencies(
    rmd_x8_90_driver_test
    Catch2
    ${dependencies}
  )
  ament_add_test(
    rmd_x8_90_driver_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/rmd_x8_90_driver_test -r junit -o test_results/${PROJECT_NAME}/rmd_x8_90_driver_test.xml
    ENV CATCH_CONFIG_CONSOLE_WIDTH=120
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
