cmake_minimum_required(VERSION 3.16)
project(adxl345_imu_broadcaster LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  adxl345_imu_broadcaster
  SHARED
  src/adxl345_imu.cpp
  src/adxl345.cpp
)
target_compile_features(adxl345_imu_broadcaster PUBLIC cxx_std_17)
target_include_directories(adxl345_imu_broadcaster PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
$<INSTALL_INTERFACE:include/adxl345_imu_broadcaster>
)
ament_target_dependencies(
  adxl345_imu_broadcaster PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link i2c library to adxl345_imu_broadcaster target
target_link_libraries(adxl345_imu_broadcaster PUBLIC i2c)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ADXL345_IMU_BROADCASTER_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface adxl345_imu_broadcaster.xml)

# INSTALL
install(
  DIRECTORY src/include/
  DESTINATION include/adxl345_imu_broadcaster
)
install(TARGETS adxl345_imu_broadcaster
  EXPORT export_adxl345_imu_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_adxl345_imu_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
