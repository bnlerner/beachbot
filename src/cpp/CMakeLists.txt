cmake_minimum_required(VERSION 3.20)
project(beachbot_cpp)

# Set C++23 standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set compiler flags for C++23
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if(APPLE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++23 -stdlib=libc++")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++23")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++23")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++23 -stdlib=libc++")
endif()

# Enable compile_commands.json generation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Find source files
file(GLOB_RECURSE IPC_SOURCES "ipc/*.cpp" "ipc/*.hpp" "ipc/*.h")
set(MAIN_SOURCE "main.cpp")
set(TEST_REQ_IPC_SOURCE "tests/request_ipc_communication.cpp")

# Create a library for IPC that both executables can link against
add_library(beachbot_ipc ${IPC_SOURCES})

# Main executable (contains main.cpp)
add_executable(beachbot_cpp ${MAIN_SOURCE})
target_link_libraries(beachbot_cpp PRIVATE beachbot_ipc)

# Test executable for request IPC
add_executable(request_ipc_communication ${TEST_REQ_IPC_SOURCE})
target_link_libraries(request_ipc_communication PRIVATE beachbot_ipc)

# Platform-specific configurations
if(APPLE)
    # MacOS specific settings
    target_compile_definitions(beachbot_cpp PRIVATE OS_MACOS=1)
    target_compile_definitions(request_ipc_communication PRIVATE OS_MACOS=1)
elseif(UNIX AND NOT APPLE)
    # Linux specific settings
    target_compile_definitions(beachbot_cpp PRIVATE OS_LINUX=1)
    target_compile_definitions(request_ipc_communication PRIVATE OS_LINUX=1)
endif()

# Threads linkage (needed for std::thread/pthread symbols)
find_package(Threads REQUIRED)
target_link_libraries(beachbot_cpp PRIVATE Threads::Threads)
target_link_libraries(request_ipc_communication PRIVATE Threads::Threads)

# Set output directory to main project build folder
set_target_properties(beachbot_cpp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../build/bin"
)
set_target_properties(request_ipc_communication PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../build/bin"
)

# Enable debug information
set(CMAKE_BUILD_TYPE Debug)
