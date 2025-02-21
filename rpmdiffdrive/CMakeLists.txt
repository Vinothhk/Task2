cmake_minimum_required(VERSION 3.8)
project(rpmdiffdrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(gazebo_ros REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-common6 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(rcl_interfaces REQUIRED)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-common6
                 COMPONENTS profiler
                 REQUIRED)
set(GZ_COMMON_VER ${gz-common6_VERSION_MAJOR})

include_directories(
        include
        )

add_library(CustomDiffDrive SHARED src/CustomDiffDrive.cc)
# target_link_libraries(RpmDiffDrive ${GAZEBO_LIBRARIES})
target_link_libraries(CustomDiffDrive
  gz-sim8::gz-sim8
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)

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


install(
  DIRECTORY
  plugins
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
