/**
 * @file ros2_zenoh_bridge.h
 * @brief ROS2-Zenoh Bridge for seamless communication between ROS2 and Zenoh
 * 
 * This file provides functions to bridge communication between ROS2 topics
 * and Zenoh topics, allowing ROS2 nodes to communicate with Zenoh nodes.
 */

#ifndef ROS2_ZENOH_BRIDGE_H
#define ROS2_ZENOH_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef struct ros2_zenoh_bridge_t ros2_zenoh_bridge_t;

// Bridge configuration
typedef struct {
    const char* zenoh_endpoint;        // Zenoh router endpoint (e.g., "tcp/172.18.0.2:7447")
    const char* ros2_namespace;        // ROS2 namespace (e.g., "/")
    const char* zenoh_prefix;          // Zenoh topic prefix (e.g., "ros2")
    bool verbose;                      // Enable verbose logging
    uint32_t qos_reliability;         // QoS reliability (0=best_effort, 1=reliable)
    uint32_t qos_durability;          // QoS durability (0=volatile, 1=transient_local, 2=transient, 3=persistent)
} ros2_zenoh_bridge_config_t;

// Default bridge configuration
#define ROS2_ZENOH_BRIDGE_DEFAULT_CONFIG { \
    .zenoh_endpoint = "tcp/172.18.0.2:7447", \
    .ros2_namespace = "/", \
    .zenoh_prefix = "ros2", \
    .verbose = false, \
    .qos_reliability = 1, \
    .qos_durability = 2 \
}

// Bridge creation and management
ros2_zenoh_bridge_t* ros2_zenoh_bridge_create(const ros2_zenoh_bridge_config_t* config);
int ros2_zenoh_bridge_destroy(ros2_zenoh_bridge_t* bridge);
bool ros2_zenoh_bridge_is_valid(const ros2_zenoh_bridge_t* bridge);

// Topic mapping functions
const char* ros2_zenoh_bridge_ros2_to_zenoh_topic(const char* ros2_topic, const char* prefix);
const char* ros2_zenoh_bridge_zenoh_to_ros2_topic(const char* zenoh_topic, const char* prefix);

// Message conversion functions
typedef struct {
    const char* message_type;          // ROS2 message type (e.g., "geometry_msgs/msg/Twist")
    const char* type_hash;            // ROS2 type hash (RIHS01 format)
    size_t message_size;              // Size of the message in bytes
} ros2_message_info_t;

// Get message information for common ROS2 message types
const ros2_message_info_t* ros2_zenoh_bridge_get_message_info(const char* message_type);

// CDR serialization helpers
int ros2_zenoh_bridge_serialize_twist(const void* twist_msg, uint8_t* buffer, size_t buffer_size, size_t* serialized_size);
int ros2_zenoh_bridge_deserialize_twist(const uint8_t* buffer, size_t buffer_size, void* twist_msg);

// Bridge operation modes
typedef enum {
    ROS2_ZENOH_BRIDGE_MODE_ROS2_TO_ZENOH = 1,    // Subscribe to ROS2, publish to Zenoh
    ROS2_ZENOH_BRIDGE_MODE_ZENOH_TO_ROS2 = 2,    // Subscribe to Zenoh, publish to ROS2
    ROS2_ZENOH_BRIDGE_MODE_BIDIRECTIONAL = 3      // Both directions
} ros2_zenoh_bridge_mode_t;

// Bridge operations
int ros2_zenoh_bridge_add_topic_mapping(
    ros2_zenoh_bridge_t* bridge,
    const char* ros2_topic,
    const char* message_type,
    ros2_zenoh_bridge_mode_t mode
);

int ros2_zenoh_bridge_start(ros2_zenoh_bridge_t* bridge);
int ros2_zenoh_bridge_stop(ros2_zenoh_bridge_t* bridge);

// Accessor functions
void* ros2_zenoh_bridge_get_zenoh_node(ros2_zenoh_bridge_t* bridge);

// Utility functions
int ros2_zenoh_bridge_sleep_ms(uint32_t milliseconds);
const char* ros2_zenoh_bridge_get_error_string(int error_code);

// Error codes
typedef enum {
    ROS2_ZENOH_BRIDGE_OK = 0,
    ROS2_ZENOH_BRIDGE_ERROR_INVALID_ARGUMENT = -1,
    ROS2_ZENOH_BRIDGE_ERROR_INITIALIZATION_FAILED = -2,
    ROS2_ZENOH_BRIDGE_ERROR_TOPIC_MAPPING_FAILED = -3,
    ROS2_ZENOH_BRIDGE_ERROR_MESSAGE_CONVERSION_FAILED = -4,
    ROS2_ZENOH_BRIDGE_ERROR_SERIALIZATION_FAILED = -5,
    ROS2_ZENOH_BRIDGE_ERROR_BRIDGE_NOT_VALID = -6,
    ROS2_ZENOH_BRIDGE_ERROR_MEMORY_ALLOCATION_FAILED = -7
} ros2_zenoh_bridge_error_t;

#ifdef __cplusplus
}
#endif

#endif // ROS2_ZENOH_BRIDGE_H
