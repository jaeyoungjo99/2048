cmake_minimum_required(VERSION 3.10)
project(game_2048)

# C++14 지정 (add_compile_options 대신에 권장되는 방법)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 커맨드(json) 출력
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# MinGW에서 정적으로 빌드하도록 플래그 세팅
if(MINGW)
    # C++ 컴파일러 플래그
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static -static-libgcc -static-libstdc++")
    
    # 링커 플래그
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")

    # freeglut을 정적으로 사용할 경우 필요할 수 있음
    add_definitions(-DGLEW_STATIC -DFREEGLUT_STATIC)
endif()

# OpenGL 찾기
find_package(OpenGL REQUIRED)

# GLFW를 pkg-config 대신, 직접 정적 라이브러리를 가리키게 할 수도 있음
# (pkg_check_modules 쓸 경우, 동적 라이브러리가 설정될 가능성이 큼)
# # 빌드해둔 정적 glfw가 있다면 예: /mingw64/lib/libglfw3.a
# set(GLFW_LIB "/mingw64/lib/libglfw3.a")

# # "s"가 붙은 GLEW 라이브러리가 정적판일 가능성이 큼
# set(GLEW_LIB "/mingw64/lib/libglew32s.a")

# # freeglut 정적판 이름은 보통 libfreeglut_static.a
# set(GLUT_LIB "/mingw64/lib/libfreeglut_static.a")

# GLFW 라이브러리 경로를 윈도우 스타일로
set(GLFW_LIB "C:/msys64/mingw64/lib/libglfw3.a")

# GLEW 라이브러리도 마찬가지
set(GLEW_LIB "C:/msys64/mingw64/lib/libglew32s.a")

# freeglut 정적 라이브러리가 있다면:
set(GLUT_LIB "C:/msys64/mingw64/lib/libfreeglut_static.a")


# include 디렉토리 설정
include_directories(
  include
  ${OPENGL_INCLUDE_DIRS}
  /mingw64/include
)

# 실행 파일
add_executable(game_2048
  src/main.cpp
  src/window.cpp
  src/game_manager.cpp
)

# 링커에 라이브러리 지정
target_link_libraries(game_2048
  ${OPENGL_LIBRARIES}
  ${GLFW_LIB}
  ${GLEW_LIB}
  ${GLUT_LIB}

  # freeglut이나 glfw가 내부적으로 필요로 하는 Windows 시스템 라이브러리들
  -lgdi32
  -lwinmm
  -lopengl32
  -lglu32
  -luser32
  -lkernel32
)
