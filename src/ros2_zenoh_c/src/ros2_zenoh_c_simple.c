/**
 * @file ros2_zenoh_c_simple.c
 * @brief Simplified implementation of ros2_zenoh_c library for Buildroot
 */

#include "ros2_zenoh_c/ros2_zenoh_c.h"
#include "ros2_zenoh_c/ros2_messages.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifdef ZENOH_AVAILABLE
#include <zenoh.h>
#endif

#ifdef MICROCDR_AVAILABLE
#include <ucdr/microcdr.h>
#endif

// Simplified internal structures
struct ros2_zenoh_node {
    char name[256];
    bool initialized;
#ifdef ZENOH_AVAILABLE
    z_owned_session_t session;
#endif
};

struct ros2_zenoh_publisher {
    ros2_zenoh_node_t* node;
    char topic[256];
    char message_type[256];
    size_t message_size;
#ifdef ZENOH_AVAILABLE
    z_owned_keyexpr_t keyexpr;
    z_owned_publisher_t publisher;
#endif
};

struct ros2_zenoh_subscriber {
    ros2_zenoh_node_t* node;
    char topic[256];
    char message_type[256];
    size_t message_size;
    ros2_zenoh_callback_t callback;
    void* user_data;
#ifdef ZENOH_AVAILABLE
    z_owned_keyexpr_t keyexpr;
    z_owned_subscriber_t subscriber;
#endif
};

// Error string mapping
static const char* error_strings[] = {
    "Success",
    "Invalid argument",
    "Memory allocation failed",
    "Zenoh error",
    "Serialization error",
    "Deserialization error"
};

// Public API implementation
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
    (*node)->initialized = false;

#ifdef ZENOH_AVAILABLE
    // For now, just mark as initialized without actually connecting to Zenoh
    // This avoids the complex ownership patterns
    (*node)->initialized = true;
#else
    (*node)->initialized = true;
#endif

    return ROS2_ZENOH_OK;
}

ros2_zenoh_ret_t ros2_zenoh_node_destroy(ros2_zenoh_node_t* node) {
    if (!node) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

#ifdef ZENOH_AVAILABLE
    // Clean up Zenoh resources if needed
    if (node->initialized) {
        // z_close(&node->session); // Skip for now due to ownership complexity
    }
#endif

    free(node);
    return ROS2_ZENOH_OK;
}

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

#ifdef ZENOH_AVAILABLE
    // For now, skip Zenoh publisher creation to avoid API complexity
    // This will be a stub implementation
#endif

    return ROS2_ZENOH_OK;
}

ros2_zenoh_ret_t ros2_zenoh_publish(
    ros2_zenoh_publisher_t* publisher,
    const void* message
) {
    if (!publisher || !message) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

#ifdef MICROCDR_AVAILABLE
    // Serialize message using Micro-CDR
    uint8_t buffer[1024];
    ucdrBuffer ucdr_buffer;
    ucdr_init_buffer(&ucdr_buffer, buffer, sizeof(buffer));

    // Serialize based on message type
    bool success = false;
    if (strcmp(publisher->message_type, "geometry_msgs/msg/Twist") == 0) {
        success = ros2_twist_serialize((const ros2_twist_t*)message, &ucdr_buffer);
    } else if (strcmp(publisher->message_type, "geometry_msgs/msg/Vector3") == 0) {
        success = ros2_vector3_serialize((const ros2_vector3_t*)message, &ucdr_buffer);
    }
    // Add more message types as needed

    if (!success) {
        return ROS2_ZENOH_ERROR_SERIALIZATION;
    }

    // For now, just print the serialized data (stub implementation)
    size_t data_size = ucdr_buffer_length(&ucdr_buffer);
    printf("Publishing %zu bytes to topic '%s'\n", data_size, publisher->topic);
    
#ifdef ZENOH_AVAILABLE
    // TODO: Actually publish via Zenoh when API is figured out
#endif

#else
    printf("Publishing message to topic '%s' (no serialization available)\n", publisher->topic);
#endif

    return ROS2_ZENOH_OK;
}

ros2_zenoh_ret_t ros2_zenoh_publisher_destroy(ros2_zenoh_publisher_t* publisher) {
    if (!publisher) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

#ifdef ZENOH_AVAILABLE
    // Clean up Zenoh resources if needed
#endif

    free(publisher);
    return ROS2_ZENOH_OK;
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

#ifdef ZENOH_AVAILABLE
    // For now, skip Zenoh subscriber creation to avoid API complexity
#endif

    return ROS2_ZENOH_OK;
}

ros2_zenoh_ret_t ros2_zenoh_subscriber_destroy(ros2_zenoh_subscriber_t* subscriber) {
    if (!subscriber) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

#ifdef ZENOH_AVAILABLE
    // Clean up Zenoh resources if needed
#endif

    free(subscriber);
    return ROS2_ZENOH_OK;
}

const char* ros2_zenoh_get_error_string(ros2_zenoh_ret_t error_code) {
    if (error_code < 0 || error_code >= (int)(sizeof(error_strings) / sizeof(error_strings[0]))) {
        return "Unknown error";
    }
    return error_strings[error_code];
}
