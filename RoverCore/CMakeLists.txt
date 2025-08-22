cmake_minimum_required(VERSION 3.16)
project(RoverCore LANGUAGES C CXX)

# ---- Options ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---- Source layout ----
set(PROJECT_SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(PROJECT_INC_DIR ${CMAKE_SOURCE_DIR}/include)
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS ${PROJECT_SRC_DIR}/*.cpp)
include_directories(${PROJECT_INC_DIR})
add_executable(rover_core ${SRC_FILES})

# ---- Output dir (./build/bin) ----
set_target_properties(rover_core PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---- Common deps ----
find_package(Threads REQUIRED)
target_link_libraries(rover_core PRIVATE Threads::Threads)

# ---- Platform-specific bits ----
if(WIN32)
  # MSVC warnings & runtime
  if(MSVC)
    target_compile_options(rover_core PRIVATE /W4 /permissive- /Zc:preprocessor /EHsc)
    # Optionally use static runtime (no MSVC redistributable needed)
    option(MSVC_STATIC_RUNTIME "Link static MSVC runtime" OFF)
    if(MSVC_STATIC_RUNTIME)
      # CMake >= 3.15
      set_property(TARGET rover_core PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    endif()
  else()
    # MinGW/Clang-cl etc.
    target_compile_options(rover_core PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  # WinSock for networking (harmless if unused)
  target_link_libraries(rover_core PRIVATE ws2_32)

  # Helpful MSVC define for <cmath> constants
  target_compile_definitions(rover_core PRIVATE _USE_MATH_DEFINES)

elseif(APPLE)
  # Clang on macOS
  target_compile_options(rover_core PRIVATE -Wall -Wextra -Wpedantic)
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    target_compile_options(rover_core PRIVATE -O2 -pipe)
  endif()
  # PIC is default on macOS, but set anyway for libs consistency
  set_target_properties(rover_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

  # Try to enable local CPU tuning (safe on native builds)
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-march=native" HAS_MARCH_NATIVE)
  if(HAS_MARCH_NATIVE)
    target_compile_options(rover_core PRIVATE -march=native)
  endif()

elseif(UNIX)
  # Linux (GCC/Clang)
  target_compile_options(rover_core PRIVATE -Wall -Wextra -Wpedantic)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Slightly stricter on GCC
    target_compile_options(rover_core PRIVATE -Wconversion -Wno-unused-parameter)
  endif()
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    target_compile_options(rover_core PRIVATE -O2 -pipe)
  endif()
  set_target_properties(rover_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-march=native" HAS_MARCH_NATIVE)
  if(HAS_MARCH_NATIVE)
    target_compile_options(rover_core PRIVATE -march=native)
  endif()
endif()

# ---- Nice defaults for smaller bins in Release ----
if(CMAKE_BUILD_TYPE MATCHES "Release")
  if(NOT WIN32)
    target_link_options(rover_core PRIVATE -s) # strip on Unix
  endif()
endif()
