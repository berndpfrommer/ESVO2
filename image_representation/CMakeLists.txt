cmake_minimum_required(VERSION 3.16)
project(esvo2_image_representation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")


find_package(ament_cmake REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(glog REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)

#
# ---------------- library ----------------
#
add_library(${PROJECT_NAME} SHARED
    src/ImageRepresentation.cpp
)

target_link_libraries(${PROJECT_NAME}
   ${OpenCV_LIBRARIES}
   image_transport::image_transport
   cv_bridge::cv_bridge
   ${std_msgs_TARGETS}
   ${event_camera_msgs_TARGETS}
   ${event_camera_codecs_TARGETS}
   Eigen3::Eigen
   rclcpp::rclcpp
   rclcpp_components::component
   glog::glog
   yaml-cpp::yaml-cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        include
)
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::ImageRepresentation")

install(TARGETS
   ${PROJECT_NAME}
   DESTINATION lib/${PROJECT_NAME}/)

#
# ------------------ executable ----------------------------
#
add_executable(${PROJECT_NAME}_node
   src/image_representation_node.cpp)

   target_link_libraries(${PROJECT_NAME}_node
      ${PROJECT_NAME}
   )
target_include_directories(${PROJECT_NAME}_node
    PRIVATE
        include
)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()
endif()

ament_package()
