cmake_minimum_required(VERSION 3.16)
project(ros2_zenoh_pico VERSION 0.1.0 LANGUAGES C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(microcdr REQUIRED)

# Find zenoh-pico manually
find_library(ZENOH_PICO_LIB zenohpico PATHS /home/ubuntu/ws/install/lib)
find_path(ZENOH_PICO_INCLUDE_DIR zenoh-pico.h PATHS /home/ubuntu/ws/install/include)

if(NOT ZENOH_PICO_LIB OR NOT ZENOH_PICO_INCLUDE_DIR)
    message(FATAL_ERROR "zenoh-pico library not found")
endif()

# Include directories
include_directories(include)
include_directories(${ZENOH_PICO_INCLUDE_DIR})

# Source files
set(SOURCES
    src/ros2_zenoh_pico_node.c
    src/ros2_zenoh_pico_publisher.c
    src/ros2_zenoh_pico_subscriber.c
    src/ros2_zenoh_pico_client.c
    src/ros2_zenoh_pico_service.c
    src/ros2_zenoh_bridge.c
)

# Create the static library
add_library(ros2_zenoh_pico STATIC ${SOURCES})

# Set target properties
set_target_properties(ros2_zenoh_pico PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Link libraries
target_link_libraries(ros2_zenoh_pico 
    ${ZENOH_PICO_LIB}
    microcdr
)

# Compile definitions
target_compile_definitions(ros2_zenoh_pico PRIVATE 
    MICROCDR_AVAILABLE
    ZENOH_PICO_AVAILABLE
    ZENOH_LINUX
    ZENOH_COMPILER_GCC
)

# Install targets
install(TARGETS ros2_zenoh_pico
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/ros2_zenoh_pico
)

# Install headers
install(DIRECTORY include/ros2_zenoh_pico/
    DESTINATION include/ros2_zenoh_pico
    FILES_MATCHING PATTERN "*.h"
)

# Build examples
file(GLOB EXAMPLE_SOURCES "examples/*.c")
foreach(example_src ${EXAMPLE_SOURCES})
    get_filename_component(example_name ${example_src} NAME_WE)
    add_executable(${example_name} ${example_src})
    
    # Special handling for simple_subscriber_test
    if(${example_name} STREQUAL "simple_subscriber_test")
        find_library(ZENOH_PICO_LIB zenohpico PATHS /home/ubuntu/ws/install/lib)
        target_link_libraries(${example_name} ${ZENOH_PICO_LIB})
        target_compile_definitions(${example_name} PRIVATE ZENOH_LINUX ZENOH_COMPILER_GCC)
    # Special handling for CDR examples
    elseif(${example_name} MATCHES ".*cdr.*")
        target_link_libraries(${example_name} ros2_zenoh_pico microcdr m)
        target_include_directories(${example_name} PRIVATE include)
        target_compile_definitions(${example_name} PRIVATE MICROCDR_AVAILABLE)
    # Special handling for bridge examples
    elseif(${example_name} MATCHES ".*bridge.*")
        target_link_libraries(${example_name} ros2_zenoh_pico microcdr m)
        target_include_directories(${example_name} PRIVATE include)
        target_compile_definitions(${example_name} PRIVATE MICROCDR_AVAILABLE)
    # Special handling for RMW examples
    elseif(${example_name} MATCHES ".*rmw.*")
        target_link_libraries(${example_name} ros2_zenoh_pico microcdr m)
        target_include_directories(${example_name} PRIVATE include)
        target_compile_definitions(${example_name} PRIVATE MICROCDR_AVAILABLE)
    else()
        target_link_libraries(${example_name} ros2_zenoh_pico)
        target_include_directories(${example_name} PRIVATE include)
    endif()
    
    # Install examples
    install(TARGETS ${example_name}
        RUNTIME DESTINATION bin
    )
endforeach()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(ros2_zenoh_pico)
ament_export_dependencies(microcdr)

# Add linting tests
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
