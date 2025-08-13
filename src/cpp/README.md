# C++ Components for Beachbot

This directory contains C++ components for the Beachbot project, configured for C++23.

## Build Requirements

- **Compiler**: Clang++ 15+ or GCC 13+ with C++23 support
- **CMake**: Version 3.20 or higher
- **macOS**: Xcode Command Line Tools

## Building

### Using CMake (Recommended)

```bash
cd src/cpp
mkdir -p ../../build
cd ../../build
cmake ../src/cpp
make
```

### Using Make

```bash
cd src/cpp
make
```

The executable will be built to `/Users/brianlerner/beachbot/build/bin/beachbot_cpp`

## Development Setup

The directory is configured with:

- **CMakeLists.txt**: CMake configuration for C++23
- **.clangd**: Clang Language Server configuration
- **.vscode/c_cpp_properties.json**: VS Code IntelliSense configuration
- **Makefile**: Simple make-based build system

## C++23 Features Available

This setup enables modern C++23 features including:

- Modules (experimental)
- Ranges improvements
- Pattern matching (when available)
- Enhanced constexpr capabilities
- And more...

## VS Code Integration

The workspace is configured to use:

- C++23 standard
- Clang++ compiler
- Enhanced IntelliSense with clangd
- Automatic compile_commands.json generation
