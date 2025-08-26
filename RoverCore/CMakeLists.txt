cmake_minimum_required(VERSION 3.16)
project(RoverCore LANGUAGES C CXX)

# ---- Basics ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# =========================================================
# Protobuf / gRPC
# =========================================================

# ---- Protobuf (normalize targets across platforms) ----
# Try CONFIG first (Homebrew/modern); fall back to legacy FindProtobuf and
# synthesize an imported target if needed.
find_package(Protobuf CONFIG QUIET)

set(PROTOBUF_LIB "")
set(PROTOC_EXE "")

if(Protobuf_FOUND)
  if(TARGET protobuf::libprotobuf)
    set(PROTOBUF_LIB protobuf::libprotobuf)
    set(PROTOC_EXE   protobuf::protoc)
  elseif(TARGET Protobuf::libprotobuf)
    set(PROTOBUF_LIB Protobuf::libprotobuf)
    set(PROTOC_EXE   Protobuf::protoc)
  elseif(TARGET Protobuf::Protobuf)
    set(PROTOBUF_LIB Protobuf::Protobuf)
    if(DEFINED Protobuf_PROTOC_EXECUTABLE)
      set(PROTOC_EXE ${Protobuf_PROTOC_EXECUTABLE})
    else()
      find_program(PROTOC_EXE protoc REQUIRED)
    endif()
  endif()
endif()

if(NOT PROTOBUF_LIB)
  # Legacy module
  find_package(Protobuf REQUIRED)  # populates Protobuf_LIBRARIES/INCLUDE_DIRS and maybe targets
  if(TARGET Protobuf::Protobuf)
    set(PROTOBUF_LIB Protobuf::Protobuf)
  else()
    # Create an imported interface target so we can link consistently
    add_library(Protobuf::Protobuf INTERFACE IMPORTED)
    set_target_properties(Protobuf::Protobuf PROPERTIES
      INTERFACE_LINK_LIBRARIES      "${Protobuf_LIBRARIES}"
      INTERFACE_INCLUDE_DIRECTORIES "${Protobuf_INCLUDE_DIRS}")
    set(PROTOBUF_LIB Protobuf::Protobuf)
  endif()

  if(DEFINED Protobuf_PROTOC_EXECUTABLE)
    set(PROTOC_EXE ${Protobuf_PROTOC_EXECUTABLE})
  else()
    find_program(PROTOC_EXE protoc REQUIRED)
  endif()
endif()

message(STATUS "Using Protobuf target: ${PROTOBUF_LIB}")
message(STATUS "Using protoc: ${PROTOC_EXE}")

# ---- gRPC ----
# On macOS/Homebrew there’s a CONFIG package; otherwise fall back to pkg-config.
if(APPLE)
  find_package(gRPC CONFIG REQUIRED)   # gRPC::grpc++, gRPC::grpc_cpp_plugin
else()
  find_package(gRPC CONFIG QUIET)
  if(NOT gRPC_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GRPC REQUIRED IMPORTED_TARGET grpc++ grpc)
    add_library(gRPC::grpc++ ALIAS PkgConfig::GRPC)
  endif()
endif()

# gRPC C++ plugin
if(TARGET gRPC::grpc_cpp_plugin)
  set(GRPC_CPP_PLUGIN $<TARGET_FILE:gRPC::grpc_cpp_plugin>)
else()
  find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin REQUIRED)
endif()

# =========================================================
# Proto generation (shared API)
# =========================================================
# Default to ../proto (override with -DPROTO_DIR=/abs/path)
set(PROTO_DIR "${PROTO_DIR}" CACHE PATH "Path to shared .proto files (default: ../proto)")
if(NOT PROTO_DIR)
  set(PROTO_DIR "${CMAKE_CURRENT_LIST_DIR}/../proto")
endif()
if(NOT EXISTS "${PROTO_DIR}")
  message(FATAL_ERROR "PROTO_DIR not found: ${PROTO_DIR}\nPass -DPROTO_DIR=/absolute/path/to/proto")
endif()

# Collect .proto files
file(GLOB PROTO_FILES_ABS CONFIGURE_DEPENDS "${PROTO_DIR}/*.proto")
if(NOT PROTO_FILES_ABS)
  message(FATAL_ERROR "No .proto files in ${PROTO_DIR}")
endif()

# Where generated C++ lands
set(GEN_DIR "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${GEN_DIR}")

# Build relative list for protoc (run from PROTO_DIR)
set(GENERATED_SRCS "")
set(GENERATED_HDRS "")
set(PROTO_FILES_REL "")
foreach(PABS ${PROTO_FILES_ABS})
  get_filename_component(BASE "${PABS}" NAME_WE)  # rover
  get_filename_component(REL  "${PABS}" NAME)     # rover.proto
  list(APPEND PROTO_FILES_REL "${REL}")
  list(APPEND GENERATED_SRCS "${GEN_DIR}/${BASE}.pb.cc" "${GEN_DIR}/${BASE}.grpc.pb.cc")
  list(APPEND GENERATED_HDRS "${GEN_DIR}/${BASE}.pb.h"  "${GEN_DIR}/${BASE}.grpc.pb.h")
endforeach()

# IMPORTANT: no quotes around args in COMMAND
add_custom_command(
  OUTPUT ${GENERATED_SRCS} ${GENERATED_HDRS}
  WORKING_DIRECTORY "${PROTO_DIR}"
  COMMAND ${PROTOC_EXE}
    --proto_path=${PROTO_DIR}
    --cpp_out=${GEN_DIR}
    --grpc_out=${GEN_DIR}
    --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
    ${PROTO_FILES_REL}
  DEPENDS ${PROTO_FILES_ABS}
  COMMENT "Generating gRPC C++ sources from ${PROTO_DIR}"
  VERBATIM
)

add_library(api_protos ${GENERATED_SRCS})
target_include_directories(api_protos PUBLIC "${GEN_DIR}")
target_link_libraries(api_protos PUBLIC ${PROTOBUF_LIB} gRPC::grpc++)

# =========================================================
# Your app
# =========================================================
set(PROJECT_SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(PROJECT_INC_DIR ${CMAKE_SOURCE_DIR}/include)
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS ${PROJECT_SRC_DIR}/*.cpp)
include_directories(${PROJECT_INC_DIR})
add_executable(rover_core ${SRC_FILES})
target_link_libraries(rover_core PRIVATE api_protos)

# Output dir ./build/bin
set_target_properties(rover_core PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Threads (POSIX)
find_package(Threads REQUIRED)
target_link_libraries(rover_core PRIVATE Threads::Threads)

# Platform niceties (trimmed)
if(WIN32)
  if(MSVC)
    target_compile_options(rover_core PRIVATE /W4 /permissive- /Zc:preprocessor /EHsc)
  else()
    target_compile_options(rover_core PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  target_link_libraries(rover_core PRIVATE ws2_32)
elseif(APPLE OR UNIX)
  target_compile_options(rover_core PRIVATE -Wall -Wextra -Wpedantic)
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    target_compile_options(rover_core PRIVATE -O2 -pipe)
  endif()
  set_target_properties(rover_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# Strip in Release (non-Windows)
if(CMAKE_BUILD_TYPE MATCHES "Release" AND NOT WIN32)
  target_link_options(rover_core PRIVATE)
endif()