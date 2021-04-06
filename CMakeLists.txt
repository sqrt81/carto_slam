cmake_minimum_required(VERSION 3.0.2)
project(carto_slam)

find_package(Boost REQUIRED COMPONENTS iostreams)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(cartographer REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES carto_slam
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${cartographer_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/map_modify.cpp)
add_executable(${PROJECT_NAME}_for_writing_map src/pbstream_to_coarse.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${boost_LIBRARIES}
    ${CERES_LIBRARIES}
    cartographer
)
target_link_libraries(${PROJECT_NAME}_for_writing_map
    ${catkin_LIBRARIES}
    ${boost_LIBRARIES}
    ${CERES_LIBRARIES}
    cartographer
)

