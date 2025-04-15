cmake_minimum_required(VERSION 3.5)

project(livox_hap_driver)
if("$ENV{ROS_DISTRO}" STRLESS_EQUAL "humble")
    find_package(rosidl_cmake REQUIRED)
    ament_export_dependencies(rosidl_cmake)
endif()

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
