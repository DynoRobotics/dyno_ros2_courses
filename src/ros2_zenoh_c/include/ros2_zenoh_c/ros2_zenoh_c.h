/**
 * @file ros2_zenoh_c.h
 * @brief Main header for ros2_zenoh_c library
 * 
 * This library provides ROS 2 message types and Zenoh integration
 * using Micro-CDR for serialization/deserialization.
 */

#ifndef ROS2_ZENOH_C_H
#define ROS2_ZENOH_C_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef MICROCDR_AVAILABLE
#include <ucdr/microcdr.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef struct ros2_zenoh_node ros2_zenoh_node_t;
typedef struct ros2_zenoh_publisher ros2_zenoh_publisher_t;
typedef struct ros2_zenoh_subscriber ros2_zenoh_subscriber_t;

// Message callback function type
typedef void (*ros2_zenoh_callback_t)(const void* message, void* user_data);

// Error codes
typedef enum {
    ROS2_ZENOH_OK = 0,
    ROS2_ZENOH_ERROR_INVALID_ARGUMENT = -1,
    ROS2_ZENOH_ERROR_MEMORY = -2,
    ROS2_ZENOH_ERROR_SERIALIZATION = -3,
    ROS2_ZENOH_ERROR_ZENOH = -4,
    ROS2_ZENOH_ERROR_NOT_INITIALIZED = -5
} ros2_zenoh_ret_t;

// Node functions
ros2_zenoh_ret_t ros2_zenoh_node_init(ros2_zenoh_node_t** node, const char* name);
ros2_zenoh_ret_t ros2_zenoh_node_destroy(ros2_zenoh_node_t* node);

// Publisher functions
ros2_zenoh_ret_t ros2_zenoh_create_publisher(
    ros2_zenoh_node_t* node,
    ros2_zenoh_publisher_t** publisher,
    const char* topic,
    const char* message_type,
    size_t message_size
);

ros2_zenoh_ret_t ros2_zenoh_publish(
    ros2_zenoh_publisher_t* publisher,
    const void* message
);

ros2_zenoh_ret_t ros2_zenoh_publisher_destroy(ros2_zenoh_publisher_t* publisher);

// Subscriber functions
ros2_zenoh_ret_t ros2_zenoh_create_subscriber(
    ros2_zenoh_node_t* node,
    ros2_zenoh_subscriber_t** subscriber,
    const char* topic,
    const char* message_type,
    size_t message_size,
    ros2_zenoh_callback_t callback,
    void* user_data
);

ros2_zenoh_ret_t ros2_zenoh_subscriber_destroy(ros2_zenoh_subscriber_t* subscriber);

// Utility functions
const char* ros2_zenoh_get_error_string(ros2_zenoh_ret_t error);

#ifdef __cplusplus
}
#endif

#endif // ROS2_ZENOH_C_H
