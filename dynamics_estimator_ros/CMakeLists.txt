cmake_minimum_required(VERSION 3.0.2)
project(dynamics_estimator_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(pinocchio REQUIRED)
find_package(dynamics_estimator REQUIRED)

catkin_package( 
  CATKIN_DEPENDS
  roscpp
)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
            src/observer_pub.cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  dynamics_estimator
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
