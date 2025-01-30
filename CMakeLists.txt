cmake_minimum_required(VERSION 3.10.0)
project(game_2048)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-std=c++14)

# ROS 관련 설정
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# OpenGL, GLFW, GLEW 관련 설정
find_package(OpenGL REQUIRED)
find_package(TBB REQUIRED) 
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)

# OpenGL 정책 설정
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

catkin_package(
    INCLUDE_DIRS include
#  INCLUDE_DIRS include
#  LIBRARIES trajectory
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


# include 디렉토리 설정
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIRS}
)

# 실행 파일 설정
add_executable(game_2048_node 
              src/main.cpp 
              src/game_manager.cpp)

# 라이브러리 링크
target_link_libraries(game_2048_node
  ${catkin_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLFW_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLUT_LIBRARIES} # Add GLUT libraries
  TBB::tbb
)
