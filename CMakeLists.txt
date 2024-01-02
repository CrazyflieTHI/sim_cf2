cmake_minimum_required(VERSION 3.8)
project(sim_cf2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_cyclonedds_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(xacro REQUIRED)

# Setting enable_ground_truth to true in the launch file loads
# the odometry_plugin which requires OpenCV to be installed 
# find_package(OpenCV REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# POSIX message queue wrapper

add_library(posixmq_wrapper STATIC src/posixmq_wrapper.c)

## OpenCV

# include_directories(${OpenCV_INCLUDE_DIRS})

## Protobuf ##

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf 3.0.0 REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
# Fallback to cmake_modules
find_package(Eigen QUIET)
if(NOT EIGEN_FOUND)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
else()
set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()
else()
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost 1.40 COMPONENTS system thread timer REQUIRED )

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  # Workaround for Eigen3
  ${Boost_INCLUDE_DIR}/eigen3
  ${EIGEN3_INCLUDE_DIRS}/Eigen3
)

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
  ${GAZEBO_MSG_INCLUDE_DIRS}
)

ament_export_include_directories(${EIGEN3_INCLUDE_DIRS})
ament_export_dependencies(rosidl_default_runtime geometry_msgs mav_msgs sensor_msgs rclcpp std_srvs std_msgs tf2_ros)

set(mymsgs
  msgs/MrSensorData.msg
)

# add an abstract target to actually trigger the builds
add_custom_target(media_files ALL DEPENDS ${urdf_files})

rosidl_generate_interfaces(${PROJECT_NAME}
  ${mymsgs}
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
)

file(GLOB msgs msgs/*.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(simcf_msgs SHARED ${PROTO_SRCS})
target_link_libraries(simcf_msgs ${PROTOBUF_LIBRARY} gazebo_msgs ${roscpp_LIBRARIES})

target_include_directories(simcf_msgs PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}  # This is where protobuf-generated headers are placed
)

## Plugins ##

add_library(rotors_gazebo_controller_interface SHARED src/gazebo_controller_interface.cpp)
include_directories(include)
target_link_libraries(rotors_gazebo_controller_interface simcf_msgs)
ament_target_dependencies(rotors_gazebo_controller_interface
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)

add_library(rotors_gazebo_motor_model SHARED src/gazebo_motor_model.cpp)
include_directories(include)
target_link_libraries(rotors_gazebo_motor_model simcf_msgs)
ament_target_dependencies(rotors_gazebo_motor_model
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)

add_library(rotors_gazebo_multirotor_base_plugin SHARED src/gazebo_multirotor_base_plugin.cpp)
include_directories(include)
target_link_libraries(rotors_gazebo_multirotor_base_plugin simcf_msgs)
ament_target_dependencies(rotors_gazebo_multirotor_base_plugin
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)

add_library(rotors_gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)
include_directories(include)
target_link_libraries(rotors_gazebo_imu_plugin simcf_msgs)
ament_target_dependencies(rotors_gazebo_imu_plugin
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)

add_library(rotors_gazebo_pressure_plugin SHARED src/gazebo_pressure_plugin.cpp)
include_directories(include)
target_link_libraries(rotors_gazebo_pressure_plugin simcf_msgs)
ament_target_dependencies(rotors_gazebo_pressure_plugin
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)

add_library(rotors_gazebo_magnetometer_plugin SHARED src/gazebo_magnetometer_plugin.cpp)
include_directories(include)
target_link_libraries(rotors_gazebo_magnetometer_plugin simcf_msgs)
ament_target_dependencies(rotors_gazebo_magnetometer_plugin
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)

add_library(rotors_gazebo_lps_plugin SHARED src/gazebo_lps_plugin.cpp)
include_directories(include)
target_link_libraries(rotors_gazebo_lps_plugin simcf_msgs)
ament_target_dependencies(rotors_gazebo_lps_plugin
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)

add_library(rotors_gazebo_wind_plugin SHARED src/gazebo_lps_plugin.cpp)
include_directories(include)
target_link_libraries(rotors_gazebo_wind_plugin simcf_msgs)
ament_target_dependencies(rotors_gazebo_wind_plugin
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)

add_library(rotors_gazebo_mr_plugin SHARED src/gazebo_mr_plugin.cpp)
include_directories(include)
target_link_libraries(rotors_gazebo_mr_plugin simcf_msgs)
ament_target_dependencies(rotors_gazebo_mr_plugin
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)


add_library(gazebo_sonar_plugin SHARED src/gazebo_sonar_plugin.cpp)
include_directories(include)
target_link_libraries(gazebo_sonar_plugin simcf_msgs)
ament_target_dependencies(gazebo_sonar_plugin
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)

add_library(gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)
include_directories(include)
target_link_libraries(gazebo_gps_plugin simcf_msgs)
ament_target_dependencies(gazebo_gps_plugin
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)

add_library(gazebo_cf_handler_plugin SHARED src/gazebo_cf_handler_plugin.cpp)
include_directories(include)
target_link_libraries(gazebo_cf_handler_plugin simcf_msgs posixmq_wrapper)
ament_target_dependencies(gazebo_cf_handler_plugin
  rclcpp
  rmw_cyclonedds_cpp
  std_msgs
  tf2_ros
  mav_msgs
  geometry_msgs
  sensor_msgs
)

# add_library(rotors_gazebo_odometry_plugin SHARED src/gazebo_odometry_plugin.cpp)
# include_directories(include)
# target_link_libraries(rotors_gazebo_odometry_plugin simcf_msgs ${OpenCV_LIBS})
# ament_target_dependencies(rotors_gazebo_odometry_plugin
#   rclcpp
#   rmw_cyclonedds_cpp
#   std_msgs
#   tf2_ros
#   mav_msgs
#   geometry_msgs
#   sensor_msgs
# )

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

install(TARGETS simcf_msgs
  DESTINATION lib)

install(TARGETS simcf_msgs
  DESTINATION lib)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY msgs
  DESTINATION share/${PROJECT_NAME})

install(TARGETS rotors_gazebo_controller_interface
  DESTINATION lib)

install(TARGETS rotors_gazebo_motor_model
  DESTINATION lib)

install(TARGETS rotors_gazebo_multirotor_base_plugin
  DESTINATION lib)

install(TARGETS rotors_gazebo_imu_plugin
  DESTINATION lib)

install(TARGETS rotors_gazebo_pressure_plugin
  DESTINATION lib)

install(TARGETS rotors_gazebo_magnetometer_plugin
  DESTINATION lib)

install(TARGETS rotors_gazebo_lps_plugin
  DESTINATION lib)

install(TARGETS rotors_gazebo_mr_plugin
  DESTINATION lib)

install(TARGETS rotors_gazebo_wind_plugin
  DESTINATION lib)

install(TARGETS gazebo_sonar_plugin
  DESTINATION lib)

install(TARGETS gazebo_gps_plugin
  DESTINATION lib)

install(TARGETS gazebo_cf_handler_plugin
  DESTINATION lib)

# install(TARGETS rotors_gazebo_odometry_plugin
#   DESTINATION lib)

ament_package()
