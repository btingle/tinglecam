cmake_minimum_required(VERSION 3.20)
project(clipper_path_lab VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# This project is intentionally independent from the repository's main build.
# Point these at already-downloaded source trees for an offline build, or let
# FetchContent download the two small dependencies when configuring.
set(CLIPPER2_UTILS OFF CACHE BOOL "" FORCE)
set(CLIPPER2_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CLIPPER2_TESTS OFF CACHE BOOL "" FORCE)
set(CLIPPER2_USINGZ OFF CACHE STRING "" FORCE)

if(CLIPPER2_CPP_DIR)
  add_subdirectory("${CLIPPER2_CPP_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/clipper2-build" EXCLUDE_FROM_ALL)
else()
  include(FetchContent)
  FetchContent_Declare(
    clipper2
    GIT_REPOSITORY https://github.com/AngusJohnson/Clipper2.git
    GIT_TAG Clipper2_2.0.1
    SOURCE_SUBDIR CPP
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(clipper2)
endif()

if(IMGUI_SOURCE_DIR)
  set(IMGUI_DIR "${IMGUI_SOURCE_DIR}")
else()
  include(FetchContent)
  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.9b
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(imgui)
  set(IMGUI_DIR "${imgui_SOURCE_DIR}")
endif()

find_package(SDL2 REQUIRED CONFIG)
find_package(OpenGL REQUIRED)

add_library(path_lab_imgui STATIC
  "${IMGUI_DIR}/imgui.cpp"
  "${IMGUI_DIR}/imgui_draw.cpp"
  "${IMGUI_DIR}/imgui_tables.cpp"
  "${IMGUI_DIR}/imgui_widgets.cpp"
  "${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp"
  "${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp"
)
target_include_directories(path_lab_imgui PUBLIC "${IMGUI_DIR}" "${IMGUI_DIR}/backends")
target_link_libraries(path_lab_imgui PUBLIC SDL2::SDL2 OpenGL::GL)

add_executable(clipper_path_lab main.cpp)
target_link_libraries(clipper_path_lab PRIVATE Clipper2 path_lab_imgui)
target_compile_options(clipper_path_lab PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

if(TARGET SDL2::SDL2main)
  target_link_libraries(clipper_path_lab PRIVATE SDL2::SDL2main)
endif()
