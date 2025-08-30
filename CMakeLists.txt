cmake_minimum_required(VERSION 3.14.4)
set(CMAKE_CXX_STANDARD 17)
project(ros_phoenix)

## Compile as C++17, supported in ROS 2
add_compile_options(-std=c++17)

# Set CTRE_ARCH
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf")
    set(CTRE_ARCH "raspbian")
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu")
    set(CTRE_ARCH "aarch64bionic")
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "x86_64-linux-gnu")
    set(CTRE_ARCH "x86-64")
else()
    message(FATAL_ERROR "Unsupported architecture for CTRE Phoenix: ${CMAKE_LIBRARY_ARCHITECTURE}")
endif()

set(CTRE_VERSION "5.19.4")

# Macro for downloading CTRE Libraries (keep existing macro)
macro(add_ctre_library LIB_NAME URL_NAME)
    SET(LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/ctre/${CTRE_VERSION}/${URL_NAME}")
    SET(LIB_DEST "${LIB_DIR}/linux/${CTRE_ARCH}/static/lib${LIB_NAME}.a")
    file(MAKE_DIRECTORY ${LIB_DIR})
    file(MAKE_DIRECTORY ${LIB_DIR}/ctre)

    # Download the library
    set(FILE_NAME "${URL_NAME}-${CTRE_VERSION}-linux${CTRE_ARCH}static.zip")
    set(FILE_URL "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/${URL_NAME}/${CTRE_VERSION}/${FILE_NAME}")
    set(FILE_DEST "${LIB_DIR}/${FILE_NAME}")
    if(NOT EXISTS ${FILE_DEST})
        message(STATUS "Downloading ${LIB_NAME} from ${FILE_URL}")
        file(DOWNLOAD ${FILE_URL} ${FILE_DEST})
    endif()

    # Download the headers
    SET(HEADER_NAME "${URL_NAME}-${CTRE_VERSION}-headers.zip")
    set(HEADER_URL "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/${URL_NAME}/${CTRE_VERSION}/${HEADER_NAME}")
    set(HEADER_DEST "${LIB_DIR}/${HEADER_NAME}")
    if(NOT EXISTS ${HEADER_DEST})
        message(STATUS "Downloading headers for ${LIB_NAME} from ${FILE_URL}")
        file(DOWNLOAD ${HEADER_URL} ${HEADER_DEST})
    endif()

    # Create a target for extracing the zip files
    add_custom_target(${LIB_NAME}-unzip DEPENDS ${LIB_DEST})
    add_custom_command(OUTPUT ${LIB_DEST} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${FILE_DEST}
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${HEADER_DEST}
        WORKING_DIRECTORY ${LIB_DIR}
        COMMENT "Unpacking ${LIB_NAME}"
        VERBATIM
    )

    # Add library
    add_library(${LIB_NAME} STATIC IMPORTED)
    add_dependencies(${LIB_NAME} ${LIB_NAME}-unzip)
    set_target_properties(${LIB_NAME} PROPERTIES
        IMPORTED_LOCATION ${LIB_DEST}
        INTERFACE_INCLUDE_DIRECTORIES ${LIB_DIR}
    )

    # Mark files for installation
    install (
        FILES ${LIB_DEST}
        DESTINATION lib
    )
    install (
        DIRECTORY ${LIB_DIR}/ctre
        DESTINATION include/${PROJECT_NAME}
    )
endmacro()

## Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)  # NEW
find_package(rcl_interfaces REQUIRED) 

## System dependencies
find_package(Threads REQUIRED)

## Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MotorControl.msg"
    "msg/MotorStatus.msg"
    "msg/ArmVelocities.msg"
    "msg/TalonConfig.msg"
    DEPENDENCIES
    std_msgs
)

set(PHOENIX_LIBRARIES
    CTRE_Phoenix
    CTRE_PhoenixCCI
    CTRE_PhoenixDiagnostics
    CTRE_PhoenixPlatform_socketcan
    CTRE_PhoenixCanutils
    CTRE_PhoenixCore
)

# Download the CTRE libraries
add_ctre_library(CTRE_Phoenix                   api-cpp)
add_ctre_library(CTRE_PhoenixCCI                cci)
add_ctre_library(CTRE_PhoenixDiagnostics        diagnostics)
add_ctre_library(CTRE_PhoenixPlatform_socketcan platform-socketcan)
add_ctre_library(CTRE_PhoenixCanutils           canutils)
add_ctre_library(CTRE_PhoenixCore               core)

## Build executables
# Comment out ROS 1 executables (migrate later)
# add_executable(ros_phoenix_cmd_example ${PROJECT_SOURCE_DIR}/src/CmdExmaple.cpp)
# ament_target_dependencies(ros_phoenix_cmd_example rclcpp std_msgs geometry_msgs)
# add_executable(ros_phoenix_fd_teleop ${PROJECT_SOURCE_DIR}/src/FDTeleop.cpp)
# ament_target_dependencies(ros_phoenix_fd_teleop rclcpp std_msgs geometry_msgs)
# add_executable(ros_phoenix_arm_command ${PROJECT_SOURCE_DIR}/src/ArmCommand.cpp)
# ament_target_dependencies(ros_phoenix_arm_command rclcpp std_msgs geometry_msgs)
# add_executable(ros_phoenix_deposition_command ${PROJECT_SOURCE_DIR}/src/DepositionCommand.cpp)
# ament_target_dependencies(ros_phoenix_deposition_command rclcpp std_msgs geometry_msgs)
# add_executable(falcon_motor_control_test_node ${PROJECT_SOURCE_DIR}/src/falcon_motor_control_test.cpp)
# ament_target_dependencies(falcon_motor_control_test_node rclcpp std_msgs geometry_msgs)
# add_executable(fake_ros_phoenix_node
#   ${PROJECT_SOURCE_DIR}/src/FakeTalonNode.cpp
#   ${PROJECT_SOURCE_DIR}/src/FakeMotorControl.cpp
# )
# ament_target_dependencies(fake_ros_phoenix_node rclcpp std_msgs geometry_msgs)

add_executable(falcon_motor_control_node ${PROJECT_SOURCE_DIR}/src/falcon_motor_control.cpp)
ament_target_dependencies(falcon_motor_control_node rclcpp std_msgs geometry_msgs sensor_msgs)
target_include_directories(falcon_motor_control_node PUBLIC ${PROJECT_SOURCE_DIR}/include)
rosidl_target_interfaces(falcon_motor_control_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(ros_phoenix_node
  ${PROJECT_SOURCE_DIR}/src/TalonNode.cpp
  ${PROJECT_SOURCE_DIR}/src/MotorControl.cpp
)
ament_target_dependencies(ros_phoenix_node rclcpp std_msgs geometry_msgs sensor_msgs)
target_include_directories(ros_phoenix_node PUBLIC ${PROJECT_SOURCE_DIR}/include)
rosidl_target_interfaces(ros_phoenix_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

## Link libraries
# Remove links for the ROS 1 targets you commented out
target_link_libraries(falcon_motor_control_node Threads::Threads ${PHOENIX_LIBRARIES})
target_link_libraries(ros_phoenix_node Threads::Threads ${PHOENIX_LIBRARIES})

## Install targets
install(TARGETS 
  ros_phoenix_node 
  falcon_motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/box_testing.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ros_phoenix/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    PATTERN ".svn" EXCLUDE
)

ament_package()