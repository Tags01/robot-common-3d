
# Note the higher version of CMAKE, needed for indirect calls (released in 2020)
cmake_minimum_required(VERSION 3.19)
project(pcl_utilities)

# Use new policy with interprocedural optimization in this file
cmake_policy(SET CMP0069 NEW)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)
# set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)

# PCL may fail to link with certain environments where
# `PCL_ROOT` becomes set to '/'.
# See: https://github.com/PointCloudLibrary/pcl/issues/4661
#
# If PCL is not found:
# - check if PCL is installed
# - check PCL_ROOT is not '/' with: `variable_watch(PCL_ROOT)`
#
# Fix by setting PCL_DIR to the path of the PCL cmake file:
set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)
include(build_target_option_flags.cmake)

find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_utility_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

set(THIS_PACKAGE_DEPS
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  pcl_utility_msgs
  PCL
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# build all files
add_library(${PROJECT_NAME} INTERFACE)

add_executable(voxel_grid_filter_service src/voxel_grid_filter_service.cpp)
add_executable(concatenate_point_cloud_service src/concatenate_point_cloud_service.cpp)

set(THIS_PACKAGE_EXECS
  voxel_grid_filter_service
  concatenate_point_cloud_service
)

foreach(TARGET ${THIS_PACKAGE_EXECS})
  ament_target_dependencies(${TARGET} PUBLIC ${THIS_PACKAGE_DEPS})

  botf_target_configurations(
    ${TARGET} PUBLIC FUNCTIONS ${BTOF_DEFAULT} btof_add_sanitizers FAILURE ${FAILURE})
  message(WARNING "${FAILURE}")
endforeach()

if(BUILD_TESTING)
  # All testing and visualization files are not produced if not in
  # BUILT_TESTING. Launch files will still exist but will fail to launch nodes

  SET(
    THIS_PACKAGE_TEST_DEPS
    rclcpp
    sensor_msgs
    pcl_ros
    pcl_utility_msgs
  )

  add_executable(simple_test_voxel_grid_filter src/tests/simple_test_voxel_grid_filter.cpp)
  add_executable(simple_test_concatenate_point_cloud src/tests/simple_test_concatenate_point_cloud.cpp)

  ament_target_dependencies(simple_test_voxel_grid_filter ${THIS_PACKAGE_TEST_DEPS})
  ament_target_dependencies(simple_test_concatenate_point_cloud ${THIS_PACKAGE_TEST_DEPS})

  list(APPEND THIS_PACKAGE_EXECS
    simple_test_voxel_grid_filter
    simple_test_concatenate_point_cloud
  )

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

#[[
  This is a workaround for a issue within Clang and BOOST MPL.
  BOOST MPL invokes undefined behavior by casting an integer to an enum
  value that does not exist. PCL uses Boost MPL so many header files do not
  compile. This should be fixed in the next release of Boost, version 1.86.0

  Clang failed to diagnose the issue until version 18.0.0
  after which it was a error. This detects the issue and opts out of the check

  NOTE: Compiler's that use Clang internally but that use different versioning
  always apply the flag do to a lack of documentation. A warning may

  See for more info: https://github.com/boostorg/mpl/issues/69
]]#
if ((CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND
    CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 16.0.0) OR
    CMAKE_CXX_COMPILER_ID MATCHES ${BTOF_CLANGLIKE_MATCH_REGEX})
  find_package(Boost)
  if (Boost_FOUND AND Boost_VERSION VERSION_LESS_EQUAL 1.86.0)
    foreach (EXECUTABLE ${THIS_PACKAGE_EXECS})
      target_compile_options(${EXECUTABLE} PUBLIC -Wno-enum-constexpr-conversion)
    endforeach()
  endif()
endif()

install(TARGETS
  ${THIS_PACKAGE_EXECS}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Note the use of enviorment variables for AddressSanitizer,
# Can conflict with other libraries if they also use build with address sanitizer
# options. But could help avoid an extra setup.

# Only sets the ASAN_OPTIONS variable if not set, will not overidde user options
# Gets sourced during `. install/setup.sh`
ament_environment_hooks(sanitizer.dsv)

ament_export_targets("export_${PROJECT_NAME}")
ament_package()
