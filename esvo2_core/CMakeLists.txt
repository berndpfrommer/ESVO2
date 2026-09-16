cmake_minimum_required(VERSION 3.16)
project(esvo2_core)

# explicitly set std=c++14 to remove errors from pcl library

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
# set(CMAKE_CXX_FLAGS "-O3")

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(events_repackaging_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(minkindr REQUIRED)
find_package(minkindr_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(HEADERS
  include/esvo2_core/core/DepthFusion.h
  include/esvo2_core/core/DepthRegularization.h
  include/esvo2_core/core/DepthProblem.h
  include/esvo2_core/core/DepthProblemSolver.h
  include/esvo2_core/core/EventBM.h
  include/esvo2_core/core/RegProblemLM.h
  include/esvo2_core/core/RegProblemSolverLM.h
  include/esvo2_core/core/BackendOptimization.h
  include/esvo2_core/factor/OptimizationFunctor.h
  include/esvo2_core/container/CameraSystem.h
  include/esvo2_core/container/DepthPoint.h
  include/esvo2_core/container/EventMatchPair.h
  include/esvo2_core/container/SmartGrid.h
  include/esvo2_core/container/DepthMap.h
  include/esvo2_core/container/ResidualItem.h
  include/esvo2_core/container/TimeSurfaceObservation.h
  include/esvo2_core/tools/Visualization.h
  include/esvo2_core/tools/utils.h
  include/esvo2_core/tools/TicToc.h
  include/esvo2_core/tools/sobel.h
  include/esvo2_core/tools/cayley.h
  include/esvo2_core/tools/params_helper.h

  include/esvo2_core/factor/utility.h

  include/esvo2_core/factor/pose_local_parameterization.h
  )

set(SOURCES
  src/core/DepthFusion.cpp
  src/core/DepthRegularization.cpp
  src/core/DepthProblem.cpp
  src/core/DepthProblemSolver.cpp
  src/core/EventBM.cpp
  src/core/RegProblemLM.cpp
  src/core/RegProblemSolverLM.cpp
  src/core/BackendOptimization.cpp
  src/container/CameraSystem.cpp
  src/container/DepthPoint.cpp
  src/container/ResidualItem.cpp
  src/tools/Visualization.cpp
  src/tools/sobel.cpp
  src/tools/cayley.cpp
  )

add_library(${PROJECT_NAME}_LIB ${SOURCES})
target_link_libraries(${PROJECT_NAME}_LIB PUBLIC Ceres::ceres 
  ${OpenCV_LIBRARIES} fmt::fmt cv_bridge::cv_bridge Eigen3::Eigen rclcpp::rclcpp image_transport::image_transport
  tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs pcl_common pcl_ros ${events_repackaging_msgs_TARGETS} ${event_camera_msgs_TARGETS})
target_include_directories(${PROJECT_NAME}_LIB PRIVATE include PUBLIC ${minkindr_conversions_INCLUDE_DIRS} ${minkindr_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${pcl_ros_INCLUDE_DIRS})


add_library(esvo2_Mapping src/esvo2_Mapping.cpp)
target_link_libraries(esvo2_Mapping ${PROJECT_NAME}_LIB image_transport::image_transport ${events_repackaging_msgs_TARGETS} ${event_camera_msgs_TARGETS} ${event_camera_codecs_TARGETS})
target_include_directories(esvo2_Mapping PRIVATE include)

# Node esvo2_Mapping
# cs_add_executable(esvo2_Mapping src/esvo2_MappingNode.cpp
#    src/esvo2_Mapping.cpp include/esvo2_core/esvo2_Mapping.h)
# target_link_libraries(esvo2_Mapping ${PROJECT_NAME}_LIB
#    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES}  yaml-cpp)

# Node esvo2_Tracking
# cs_add_executable(esvo2_Tracking src/esvo2_TrackingNode.cpp
#    src/esvo2_Tracking.cpp include/esvo2_core/esvo2_Tracking.h)
# target_link_libraries(esvo2_Tracking ${PROJECT_NAME}_LIB
#    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

install(TARGETS
  ${PROJECT_NAME}_LIB
  esvo2_Mapping
  DESTINATION lib)
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
  ament_clang_format()
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()
endif()


ament_package()