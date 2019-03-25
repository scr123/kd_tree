cmake_minimum_required(VERSION 2.8.3)
project(kd_tree)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS )

catkin_package(
  INCLUDE_DIRS 
  include
  CATKIN_DEPENDS
)

add_library(
  ${PROJECT_NAME}
  INTERFACE
)

target_include_directories(
  ${PROJECT_NAME} 
  INTERFACE
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  INTERFACE 
  ${catkin_LIBRARIES}
)


###########
# K-D TREE
###########
catkin_add_gtest(kd_tree_test test/kd_tree_test.cpp)
target_include_directories(kd_tree_test
  SYSTEM PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(kd_tree_test ${catkin_LIBRARIES})
add_dependencies(kd_tree_test ${catkin_EXPORTED_TARGETS})


###########
# NODE
###########
catkin_add_gtest(node_test test/node_test.cpp)
target_include_directories(node_test
  SYSTEM PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(node_test ${catkin_LIBRARIES})
add_dependencies(node_test ${catkin_EXPORTED_TARGETS})


###########
# STATE
###########
catkin_add_gtest(state_test test/state_test.cpp)
target_include_directories(state_test
  SYSTEM PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(state_test ${catkin_LIBRARIES})
add_dependencies(state_test ${catkin_EXPORTED_TARGETS})