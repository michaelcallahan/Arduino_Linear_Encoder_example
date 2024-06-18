cmake_minimum_required(VERSION 3.16)
project(motor_hardware_interface LANGUAGES CXX)

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
  motor_hardware_interface
  SHARED
  src/motor_hardware_interface.cpp
)
target_compile_features(motor_hardware_interface PUBLIC cxx_std_17)
target_include_directories(motor_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
$<INSTALL_INTERFACE:include/motor_hardware_interface>
)
ament_target_dependencies(
  motor_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "motor_hardware_interface_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface motor_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY src/include
  DESTINATION share/motor_hardware_interface/src
)
install(
  DIRECTORY src
  DESTINATION share/motor_hardware_interface
)
install(
  DIRECTORY urdf
  DESTINATION share/motor_hardware_interface
)
install(
  DIRECTORY config
  DESTINATION share/motor_hardware_interface
)
install(
  DIRECTORY launch
  DESTINATION share/motor_hardware_interface/launch
)
#install(TARGETS motor_hardware_interface
#  EXPORT export_motor_hardware_interface
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

install(
  TARGETS motor_hardware_interface
  DESTINATION lib
)


## EXPORTS
#ament_export_targets(export_motor_hardware_interface HAS_LIBRARY_TARGET)
#ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()