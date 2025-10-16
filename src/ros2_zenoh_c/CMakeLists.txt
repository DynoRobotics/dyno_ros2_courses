cmake_minimum_required(VERSION 3.16)
project(ros2_zenoh_c VERSION 0.1.0 LANGUAGES C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)

# Try to find Zenoh C library, but don't fail if not found
# Temporarily disable Zenoh to get basic build working
# find_path(ZENOH_INCLUDE_DIR zenoh.h PATHS /opt/ros/jazzy/opt/zenoh_cpp_vendor/include)
# find_library(ZENOH_LIBRARY NAMES zenohc PATHS /opt/ros/jazzy/opt/zenoh_cpp_vendor/lib)

# if(ZENOH_INCLUDE_DIR AND ZENOH_LIBRARY)
#     add_definitions(-DZENOH_AVAILABLE)
#     message(STATUS "Found Zenoh C library: ${ZENOH_LIBRARY}")
# else()
    message(WARNING "Zenoh C library temporarily disabled. Building without Zenoh support.")
    set(ZENOH_INCLUDE_DIR "")
    set(ZENOH_LIBRARY "")
# endif()

# Find Micro-CDR (use the one we built)
find_path(MICROCDR_INCLUDE_DIR ucdr/microcdr.h PATHS /home/ubuntu/ws/install/micro-cdr/microcdr-2.0.2/include)
find_library(MICROCDR_LIBRARY NAMES microcdr PATHS /home/ubuntu/ws/install/micro-cdr/microcdr-2.0.2/lib)

if(MICROCDR_INCLUDE_DIR AND MICROCDR_LIBRARY)
    add_definitions(-DMICROCDR_AVAILABLE)
else()
    message(WARNING "Micro-CDR not found. Building without Micro-CDR support.")
    set(MICROCDR_INCLUDE_DIR "")
    set(MICROCDR_LIBRARY "")
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${ZENOH_INCLUDE_DIR})
include_directories(${MICROCDR_INCLUDE_DIR})

# Library sources
set(LIB_SOURCES
    src/ros2_zenoh_c.c
    src/ros2_messages.c
    src/ros2_zenoh_c_stubs.c
)

# Create the library
add_library(${PROJECT_NAME} STATIC ${LIB_SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} 
    ${ZENOH_LIBRARY}
    ${MICROCDR_LIBRARY}
)

# Compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE "")

# Install the library
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

# Install headers
install(DIRECTORY include/ros2_zenoh_c
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

# Examples
add_executable(publisher_example examples/publisher_example.c)
target_link_libraries(publisher_example ${PROJECT_NAME})

add_executable(subscriber_example examples/subscriber_example.c)
target_link_libraries(subscriber_example ${PROJECT_NAME})

# Install examples
install(TARGETS publisher_example subscriber_example
    RUNTIME DESTINATION bin
)

# Tests (if needed)
enable_testing()

# Package configuration
set(CPACK_PACKAGE_NAME "ros2_zenoh_c")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "ROS 2 Zenoh C Library")
set(CPACK_PACKAGE_VENDOR "ROS 2 Zenoh Team")

include(CPack)
