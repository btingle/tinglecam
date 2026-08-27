cmake_minimum_required(VERSION 3.20)
project(clipper2_wave_demo VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

# Keep third-party builds small and deterministic.
set(CLIPPER2_UTILS OFF CACHE BOOL "" FORCE)
set(CLIPPER2_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CLIPPER2_TESTS OFF CACHE BOOL "" FORCE)
set(CLIPPER2_USINGZ OFF CACHE STRING "" FORCE)
FetchContent_Declare(
  clipper2
  GIT_REPOSITORY https://github.com/AngusJohnson/Clipper2.git
  GIT_TAG Clipper2_1.5.4
  SOURCE_SUBDIR CPP
  GIT_SHALLOW TRUE
)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.92.9b
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(clipper2 imgui)

find_package(SDL2 REQUIRED CONFIG)
find_package(OpenGL REQUIRED)

add_library(wave_geometry
  src/geometry.cpp
)
target_include_directories(wave_geometry PUBLIC src)
target_link_libraries(wave_geometry PUBLIC Clipper2)
target_compile_options(wave_geometry PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_library(imgui_sdl_opengl STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_sdl_opengl PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_sdl_opengl PUBLIC SDL2::SDL2 OpenGL::GL)

add_executable(clipper2_wave_demo src/main.cpp)
target_link_libraries(clipper2_wave_demo PRIVATE wave_geometry imgui_sdl_opengl)
if(TARGET SDL2::SDL2main)
  target_link_libraries(clipper2_wave_demo PRIVATE SDL2::SDL2main)
endif()

add_custom_command(TARGET clipper2_wave_demo POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/wavetest.svg
          $<TARGET_FILE_DIR:clipper2_wave_demo>/wavetest.svg
)

enable_testing()
add_executable(wave_geometry_tests tests/geometry_tests.cpp)
target_link_libraries(wave_geometry_tests PRIVATE wave_geometry)
add_test(NAME wave_geometry_tests
         COMMAND wave_geometry_tests ${CMAKE_CURRENT_SOURCE_DIR}/wavetest.svg)

