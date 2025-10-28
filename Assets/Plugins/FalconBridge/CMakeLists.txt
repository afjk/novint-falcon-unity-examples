cmake_minimum_required(VERSION 3.10)
project(FalconBridge)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use libnifalcon from submodule
set(LIBNIFALCON_ROOT "${CMAKE_SOURCE_DIR}/../../../libnifalcon")
set(NIFALCON_INCLUDE "${LIBNIFALCON_ROOT}/include")
set(NIFALCON_LIB_DIR "${LIBNIFALCON_ROOT}/build/lib")
set(NIFALCON_LIB "${NIFALCON_LIB_DIR}/libnifalcon.dylib")

# Check if libnifalcon is built
if(NOT EXISTS "${NIFALCON_LIB}")
    message(FATAL_ERROR "libnifalcon not found at ${NIFALCON_LIB}. Please build libnifalcon first:\n"
                        "  cd ${LIBNIFALCON_ROOT}\n"
                        "  mkdir -p build && cd build\n"
                        "  cmake ..\n"
                        "  make")
endif()

# Source files
set(SOURCES
    FalconBridge.cpp
)

set(HEADERS
    FalconBridge.h
)

# Create bundle module for Unity on macOS
add_library(FalconBridge MODULE ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(FalconBridge PRIVATE ${NIFALCON_INCLUDE})

# Link libraries
target_link_libraries(FalconBridge ${NIFALCON_LIB})

# Set output properties for macOS
if(APPLE)
    set_target_properties(FalconBridge PROPERTIES
        BUNDLE TRUE
        BUNDLE_EXTENSION "bundle"
        MACOSX_BUNDLE_INFO_PLIST ""
        PREFIX ""
        SUFFIX ".bundle"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/../macOS"
    )

    # Set visibility for exported symbols
    target_compile_options(FalconBridge PRIVATE -fvisibility=hidden)

    # Enable C++11 threads
    target_compile_options(FalconBridge PRIVATE -pthread)
endif()

# For debugging
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# Print configuration info
message(STATUS "Building FalconBridge for Unity")
message(STATUS "libnifalcon library: ${NIFALCON_LIB}")
message(STATUS "libnifalcon include: ${NIFALCON_INCLUDE}")
message(STATUS "Output directory: ${CMAKE_SOURCE_DIR}/../macOS")
