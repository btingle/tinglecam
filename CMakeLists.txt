cmake_minimum_required(VERSION 3.20)
project(clipper2_wave_demo VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
include(CheckLanguage)

# Mesh preprocessing relies on the cuBQL BVH. There is deliberately no CPU
# fallback: its all-pairs containment pass made application startup quadratic.
# Prefer the active /usr/local CUDA toolkit over an older distribution nvcc.
# CUDACXX or an explicit CMAKE_CUDA_COMPILER still takes precedence.
if(NOT DEFINED CMAKE_CUDA_COMPILER AND NOT DEFINED ENV{CUDACXX})
  find_program(WAVE_LOCAL_NVCC nvcc HINTS /usr/local/cuda/bin NO_DEFAULT_PATH)
  if(WAVE_LOCAL_NVCC)
    set(CMAKE_CUDA_COMPILER "${WAVE_LOCAL_NVCC}" CACHE FILEPATH "CUDA compiler")
  endif()
endif()
if(NOT CMAKE_CUDA_ARCHITECTURES)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    set(CMAKE_CUDA_ARCHITECTURES native CACHE STRING "CUDA architectures")
  else()
    set(CMAKE_CUDA_ARCHITECTURES "70;80" CACHE STRING "CUDA architectures")
  endif()
endif()
check_language(CUDA)
if(NOT CMAKE_CUDA_COMPILER)
  message(FATAL_ERROR
    "CUDA is required: TingleCAM's mesh slicer uses the cuBQL CUDA BVH and has no CPU fallback")
endif()
enable_language(CUDA)

# Keep third-party builds small and deterministic.
set(CLIPPER2_UTILS OFF CACHE BOOL "" FORCE)
set(CLIPPER2_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CLIPPER2_TESTS OFF CACHE BOOL "" FORCE)
set(CLIPPER2_USINGZ OFF CACHE STRING "" FORCE)
FetchContent_Declare(
  clipper2
  GIT_REPOSITORY https://github.com/AngusJohnson/Clipper2.git
  GIT_TAG Clipper2_2.0.1
  SOURCE_SUBDIR CPP
  GIT_SHALLOW TRUE
)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.92.9b
  GIT_SHALLOW TRUE
)

set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
  GIT_SHALLOW TRUE
)

FetchContent_Declare(
  cubql
  GIT_REPOSITORY https://github.com/NVIDIA/cuBQL.git
  GIT_TAG 20f9db19cdb160e2b83d8f05dfdb0437fe11fe24
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(clipper2 imgui nlohmann_json)
FetchContent_MakeAvailable(cubql)

find_package(SDL2 REQUIRED CONFIG)
find_package(OpenGL REQUIRED)

add_library(wave_geometry
  src/analysis.cpp
  src/geometry.cpp
  src/mesh_slicer.cpp
  src/simulation.cpp
  src/topology.cpp
  src/routing.cpp
  src/traversal.cpp
  src/toolpath.cpp
)
target_sources(wave_geometry PRIVATE src/mesh_slicer_cuda.cu)
target_link_libraries(wave_geometry PRIVATE cuBQL_cuda_float3_static)
target_compile_definitions(wave_geometry PUBLIC WAVE_HAVE_CUDA=1)
set_target_properties(wave_geometry PROPERTIES CUDA_STANDARD 20 CUDA_STANDARD_REQUIRED ON)
target_include_directories(wave_geometry PUBLIC src)
target_link_libraries(wave_geometry PUBLIC Clipper2)
target_compile_options(wave_geometry PRIVATE
  $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:GNU,Clang>>:-Wall;-Wextra;-Wpedantic>
  $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:MSVC>>:/W4>
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

# TingleCAM: the CAM application over the same engine. The demo above stays as
# a way of looking at a wavefront generation by generation when a plan is wrong.
add_library(tinglecam_core
  src/cam/contours.cpp
  src/cam/config.cpp
  src/cam/feeds.cpp
  src/cam/job.cpp
  src/cam/machine.cpp
  src/cam/material.cpp
  src/cam/post.cpp
  src/cam/tooling.cpp
)
target_link_libraries(tinglecam_core PUBLIC wave_geometry nlohmann_json::nlohmann_json)
target_include_directories(tinglecam_core PUBLIC src)
target_compile_definitions(tinglecam_core PRIVATE
  TINGLECAM_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_compile_options(tinglecam_core PRIVATE
  $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:GNU,Clang>>:-Wall;-Wextra;-Wpedantic>
  $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:MSVC>>:/W4>
)

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

add_executable(tinglecam src/tinglecam.cpp)
target_link_libraries(tinglecam PRIVATE tinglecam_core imgui_sdl_opengl)
target_compile_definitions(tinglecam PRIVATE
  TINGLECAM_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
if(TARGET SDL2::SDL2main)
  target_link_libraries(tinglecam PRIVATE SDL2::SDL2main)
endif()

enable_testing()
add_executable(wave_geometry_tests tests/geometry_tests.cpp)
target_link_libraries(wave_geometry_tests PRIVATE wave_geometry)
add_test(NAME wave_geometry_tests
         COMMAND wave_geometry_tests ${CMAKE_CURRENT_SOURCE_DIR}/wavetest.svg
                 ${CMAKE_CURRENT_SOURCE_DIR}/objs/LsJig1.obj)

add_executable(routing_tests tests/routing_tests.cpp)
target_link_libraries(routing_tests PRIVATE wave_geometry)
add_test(NAME routing_tests COMMAND routing_tests)

add_executable(mesh_slicer_tests tests/mesh_slicer_tests.cpp)
target_link_libraries(mesh_slicer_tests PRIVATE wave_geometry)
add_test(NAME mesh_slicer_tests COMMAND mesh_slicer_tests)

add_executable(simulation_tests tests/simulation_tests.cpp)
target_link_libraries(simulation_tests PRIVATE wave_geometry)
add_test(NAME simulation_tests
         COMMAND simulation_tests ${CMAKE_CURRENT_SOURCE_DIR}/wavetest.svg)

add_executable(cam_tests tests/cam_tests.cpp)
target_link_libraries(cam_tests PRIVATE tinglecam_core)
add_test(NAME cam_tests COMMAND cam_tests ${CMAKE_CURRENT_SOURCE_DIR}/data
         ${CMAKE_CURRENT_SOURCE_DIR}/objs/final_obj/L.obj
         ${CMAKE_CURRENT_SOURCE_DIR}/objs)

add_test(NAME tinglecam_help COMMAND tinglecam --help)

add_executable(tool_library_tests tests/tool_library_tests.cpp)
target_link_libraries(tool_library_tests PRIVATE tinglecam_core imgui_sdl_opengl)
add_test(NAME tool_library_tests COMMAND tool_library_tests)

add_test(NAME demo_help COMMAND clipper2_wave_demo --help)
add_test(NAME demo_analysis_dump
         COMMAND clipper2_wave_demo ${CMAKE_CURRENT_SOURCE_DIR}/wavetest.svg
                 --mitered-waves --dump-analysis)
