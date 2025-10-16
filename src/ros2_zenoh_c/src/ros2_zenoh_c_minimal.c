/**
 * @file ros2_zenoh_c_minimal.c
 * @brief Minimal implementation of ros2_zenoh_c library (without external dependencies)
 */

#include "ros2_zenoh_c/ros2_zenoh_c.h"
#include "ros2_zenoh_c/ros2_messages.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Internal structures
struct ros2_zenoh_node {
    char name[256];
    bool initialized;
};

struct ros2_zenoh_publisher {
    ros2_zenoh_node_t* node;
    char topic[256];
    char message_type[256];
    size_t message_size;
};

struct ros2_zenoh_subscriber {
    ros2_zenoh_node_t* node;
    char topic[256];
    char message_type[256];
    size_t message_size;
    ros2_zenoh_callback_t callback;
    void* user_data;
};

// Error string mapping
static const char* error_strings[] = {
    "Success",
    "Invalid argument",
    "Memory allocation error",
    "Serialization error",
    "Zenoh error",
    "Not initialized"
};

const char* ros2_zenoh_get_error_string(ros2_zenoh_ret_t error) {
    int index = -(int)error;
    if (index >= 0 && index < (int)(sizeof(error_strings) / sizeof(error_strings[0]))) {
        return error_strings[index];
    }
    return "Unknown error";
}

// Node functions
ros2_zenoh_ret_t ros2_zenoh_node_init(ros2_zenoh_node_t** node, const char* name) {
    if (!node || !name) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

    *node = (ros2_zenoh_node_t*)malloc(sizeof(ros2_zenoh_node_t));
    if (!*node) {
        return ROS2_ZENOH_ERROR_MEMORY;
    }

    strncpy((*node)->name, name, sizeof((*node)->name) - 1);
    (*node)->name[sizeof((*node)->name) - 1] = '\0';
    (*node)->initialized = true;

    return ROS2_ZENOH_OK;
}

ros2_zenoh_ret_t ros2_zenoh_node_destroy(ros2_zenoh_node_t* node) {
    if (!node) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

    free(node);
    return ROS2_ZENOH_OK;
}

// Publisher functions
ros2_zenoh_ret_t ros2_zenoh_create_publisher(
    ros2_zenoh_node_t* node,
    ros2_zenoh_publisher_t** publisher,
    const char* topic,
    const char* message_type,
    size_t message_size
) {
    if (!node || !publisher || !topic || !message_type || !node->initialized) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

    *publisher = (ros2_zenoh_publisher_t*)malloc(sizeof(ros2_zenoh_publisher_t));
    if (!*publisher) {
        return ROS2_ZENOH_ERROR_MEMORY;
    }

    (*publisher)->node = node;
    strncpy((*publisher)->topic, topic, sizeof((*publisher)->topic) - 1);
    (*publisher)->topic[sizeof((*publisher)->topic) - 1] = '\0';
    strncpy((*publisher)->message_type, message_type, sizeof((*publisher)->message_type) - 1);
    (*publisher)->message_type[sizeof((*publisher)->message_type) - 1] = '\0';
    (*publisher)->message_size = message_size;

    return ROS2_ZENOH_OK;
}

ros2_zenoh_ret_t ros2_zenoh_publish(
    ros2_zenoh_publisher_t* publisher,
    const void* message
) {
    if (!publisher || !message) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

    // In a minimal implementation, we just return success
    // In a full implementation, this would serialize and publish via Zenoh
    printf("Publishing message of type %s to topic %s\n", 
           publisher->message_type, publisher->topic);
    
    return ROS2_ZENOH_OK;
}

ros2_zenoh_ret_t ros2_zenoh_publisher_destroy(ros2_zenoh_publisher_t* publisher) {
    if (!publisher) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

    free(publisher);
    return ROS2_ZENOH_OK;
}

// Subscriber functions
ros2_zenoh_ret_t ros2_zenoh_create_subscriber(
    ros2_zenoh_node_t* node,
    ros2_zenoh_subscriber_t** subscriber,
    const char* topic,
    const char* message_type,
    size_t message_size,
    ros2_zenoh_callback_t callback,
    void* user_data
) {
    if (!node || !subscriber || !topic || !message_type || !callback || !node->initialized) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

    *subscriber = (ros2_zenoh_subscriber_t*)malloc(sizeof(ros2_zenoh_subscriber_t));
    if (!*subscriber) {
        return ROS2_ZENOH_ERROR_MEMORY;
    }

    (*subscriber)->node = node;
    strncpy((*subscriber)->topic, topic, sizeof((*subscriber)->topic) - 1);
    (*subscriber)->topic[sizeof((*subscriber)->topic) - 1] = '\0';
    strncpy((*subscriber)->message_type, message_type, sizeof((*subscriber)->message_type) - 1);
    (*subscriber)->message_type[sizeof((*subscriber)->message_type) - 1] = '\0';
    (*subscriber)->message_size = message_size;
    (*subscriber)->callback = callback;
    (*subscriber)->user_data = user_data;

    printf("Created subscriber for topic %s with message type %s\n", 
           topic, message_type);

    return ROS2_ZENOH_OK;
}

ros2_zenoh_ret_t ros2_zenoh_subscriber_destroy(ros2_zenoh_subscriber_t* subscriber) {
    if (!subscriber) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

    free(subscriber);
    return ROS2_ZENOH_OK;
}
