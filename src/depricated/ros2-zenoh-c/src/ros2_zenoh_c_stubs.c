/**
 * @file ros2_zenoh_c_stubs.c
 * @brief Stub implementations when dependencies are not available
 */

#include "ros2_zenoh_c/ros2_zenoh_c.h"
#include "ros2_zenoh_c/ros2_messages.h"

#ifndef ZENOH_AVAILABLE
// Stub implementations when Zenoh is not available

ros2_zenoh_ret_t ros2_zenoh_node_init(ros2_zenoh_node_t** node, const char* name) {
    (void)node; (void)name;
    return ROS2_ZENOH_ERROR_ZENOH;  // Zenoh not available
}

ros2_zenoh_ret_t ros2_zenoh_node_destroy(ros2_zenoh_node_t* node) {
    (void)node;
    return ROS2_ZENOH_ERROR_ZENOH;  // Zenoh not available
}

const char* ros2_zenoh_get_error_string(ros2_zenoh_ret_t ret) {
    switch(ret) {
        case ROS2_ZENOH_OK: return "OK";
        case ROS2_ZENOH_ERROR_ZENOH: return "Zenoh not available";
        case ROS2_ZENOH_ERROR_MEMORY: return "Memory error";
        case ROS2_ZENOH_ERROR_INVALID_ARGUMENT: return "Invalid argument";
        default: return "Unknown error";
    }
}

ros2_zenoh_ret_t ros2_zenoh_create_publisher(
    ros2_zenoh_node_t* node,
    ros2_zenoh_publisher_t** publisher,
    const char* topic,
    const char* message_type,
    size_t message_size
) {
    (void)node; (void)publisher; (void)topic; (void)message_type; (void)message_size;
    return ROS2_ZENOH_ERROR_ZENOH;  // Zenoh not available
}

ros2_zenoh_ret_t ros2_zenoh_publish(
    ros2_zenoh_publisher_t* publisher,
    const void* message
) {
    (void)publisher; (void)message;
    return ROS2_ZENOH_ERROR_ZENOH;  // Zenoh not available
}

ros2_zenoh_ret_t ros2_zenoh_publisher_destroy(ros2_zenoh_publisher_t* publisher) {
    (void)publisher;
    return ROS2_ZENOH_ERROR_ZENOH;  // Zenoh not available
}

ros2_zenoh_ret_t ros2_zenoh_create_subscriber(
    ros2_zenoh_node_t* node,
    ros2_zenoh_subscriber_t** subscriber,
    const char* topic,
    const char* message_type,
    size_t message_size,
    ros2_zenoh_callback_t callback,
    void* user_data
) {
    (void)node; (void)subscriber; (void)topic; (void)message_type; 
    (void)message_size; (void)callback; (void)user_data;
    return ROS2_ZENOH_ERROR_ZENOH;  // Zenoh not available
}

ros2_zenoh_ret_t ros2_zenoh_subscriber_destroy(ros2_zenoh_subscriber_t* subscriber) {
    (void)subscriber;
    return ROS2_ZENOH_ERROR_ZENOH;  // Zenoh not available
}

#endif // ZENOH_AVAILABLE

#ifndef MICROCDR_AVAILABLE
// Stub implementations when Micro-CDR is not available

bool ros2_time_serialize(const ros2_time_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_time_deserialize(ros2_time_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_duration_serialize(const ros2_duration_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_duration_deserialize(ros2_duration_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_header_serialize(const ros2_header_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_header_deserialize(ros2_header_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_vector3_serialize(const ros2_vector3_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_vector3_deserialize(ros2_vector3_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_quaternion_serialize(const ros2_quaternion_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_quaternion_deserialize(ros2_quaternion_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_transform_serialize(const ros2_transform_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_transform_deserialize(ros2_transform_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_transform_stamped_serialize(const ros2_transform_stamped_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_transform_stamped_deserialize(ros2_transform_stamped_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_twist_serialize(const ros2_twist_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_twist_deserialize(ros2_twist_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_twist_stamped_serialize(const ros2_twist_stamped_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_twist_stamped_deserialize(ros2_twist_stamped_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_pose_serialize(const ros2_pose_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_pose_deserialize(ros2_pose_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_pose_stamped_serialize(const ros2_pose_stamped_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_pose_stamped_deserialize(ros2_pose_stamped_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_point_serialize(const ros2_point_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_point_deserialize(ros2_point_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_point_stamped_serialize(const ros2_point_stamped_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

bool ros2_point_stamped_deserialize(ros2_point_stamped_t* msg, void* buffer) {
    (void)msg; (void)buffer;
    return false;  // Micro-CDR not available
}

#endif // MICROCDR_AVAILABLE
