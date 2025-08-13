cmake_minimum_required(VERSION 3.20)
project(beachbot_cpp)

# Set C++23 standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set compiler flags for C++23
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++23 -stdlib=libc++")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++23")
endif()

# Enable compile_commands.json generation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Find source files
file(GLOB_RECURSE SOURCES "*.cpp" "*.hpp" "*.h")

# Create executable
add_executable(beachbot_cpp ${SOURCES})

# Set output directory to main project build folder
set_target_properties(beachbot_cpp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../build/bin
)

# Enable debug information
set(CMAKE_BUILD_TYPE Debug)
