cmake_minimum_required(VERSION 2.8.3)
project(md_slam)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg_hbst
  srrg2_core
  srrg2_core_ros
  srrg2_solver
)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=pedantic -fopenmp")
message("CMAKE FLAGS SET TO ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(QGLViewer REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "${PROJECT_NAME}: OpenCV version ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSRRG_MERGE_DESCRIPTORS -DSRRG_HBST_HAS_OPENCV")

catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    md_slam
  CATKIN_DEPENDS
    srrg_cmake_modules
    srrg_hbst
    srrg2_core
    srrg2_core_ros
    srrg2_solver
  DEPENDS
    SRRG_QT
    OpenCV
    QGLViewer
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${QGLVIEWER_INCLUDE_DIR}
  ${SRRG_QT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

#ia help catkin build
link_directories(${catkin_LIBRARY_DIRS})

# path to damn dynamic loader
set(DL_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/../")
add_definitions(-DMD_DL_FOLDER="${DL_FOLDER}")
message("${PROJECT_NAME}|dl_dir dir = ${DL_FOLDER}")

# srrg preprocessor definition used for tests (and apps)
set(TEST_DATA_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/../test_data/")
add_definitions(-DMD_TEST_DATA_FOLDER="${TEST_DATA_FOLDER}")
message("${PROJECT_NAME}|test_data dir = ${TEST_DATA_FOLDER}")

set(CONFIG_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/../configurations/")
add_definitions(-DMD_CONFIG_FOLDER="${CONFIG_FOLDER}")
message("${PROJECT_NAME}|configurations dir = ${CONFIG_FOLDER}")

# sources folder
add_subdirectory(src)
add_subdirectory(apps)

# unittest folder
add_subdirectory(tests)
