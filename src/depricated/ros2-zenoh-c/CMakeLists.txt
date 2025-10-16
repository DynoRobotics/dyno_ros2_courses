cmake_minimum_required(VERSION 3.16)
project(ros2_zenoh_c VERSION 0.1.0 LANGUAGES C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find required packages
find_package(microcdr REQUIRED)
find_package(zenohpico REQUIRED)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/ros2_messages.c
    src/ros2_zenoh_c_stubs.c
)

# Create the static library
add_library(ros2_zenoh_c STATIC ${SOURCES})

# Set target properties
set_target_properties(ros2_zenoh_c PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Link libraries
target_link_libraries(ros2_zenoh_c microcdr zenohpico::lib)

# Compile definitions
target_compile_definitions(ros2_zenoh_c PRIVATE MICROCDR_AVAILABLE)

# Install targets
install(TARGETS ros2_zenoh_c
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/ros2_zenoh_c
)

# Install headers
install(DIRECTORY include/ros2_zenoh_c/
    DESTINATION include/ros2_zenoh_c
    FILES_MATCHING PATTERN "*.h"
)

# Build examples
file(GLOB EXAMPLE_SOURCES "examples/*.c")
foreach(example_src ${EXAMPLE_SOURCES})
    get_filename_component(example_name ${example_src} NAME_WE)
    add_executable(${example_name} ${example_src})
    target_link_libraries(${example_name} ros2_zenoh_c)
    target_include_directories(${example_name} PRIVATE include)
    
    # Install examples
    install(TARGETS ${example_name}
        RUNTIME DESTINATION bin
    )
endforeach()
