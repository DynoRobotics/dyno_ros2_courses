/**
 * @file ros2_zenoh_c.c
 * @brief Implementation of ros2_zenoh_c library
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

// Internal structures
struct ros2_zenoh_node {
    char name[256];
#ifdef ZENOH_AVAILABLE
    z_owned_session_t session;
#endif
    bool initialized;
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
#ifdef ZENOH_AVAILABLE
    (*node)->session = NULL;
#endif
    (*node)->initialized = false;

#ifdef ZENOH_AVAILABLE
    // Initialize Zenoh session
    z_owned_config_t config;
    z_config_default(&config);
    
    z_open_options_t options;
    z_open_options_default(&options);
    
    z_result_t result = z_open(&(*node)->session, z_move(&config), &options);
    if (result != 0) {
        free(*node);
        *node = NULL;
        return ROS2_ZENOH_ERROR_ZENOH;
    }
#endif
    (*node)->initialized = true;

    return ROS2_ZENOH_OK;
}

ros2_zenoh_ret_t ros2_zenoh_node_destroy(ros2_zenoh_node_t* node) {
    if (!node) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

#ifdef ZENOH_AVAILABLE
    if (node->initialized) {
        z_close(z_move(&node->session));
    }
#endif

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

#ifndef ZENOH_AVAILABLE
    return ROS2_ZENOH_ERROR_ZENOH;  // Zenoh not available
#endif

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
    // Create Zenoh key expression
    z_result_t keyexpr_result = z_declare_keyexpr(
        z_session_loan(&node->session), 
        &(*publisher)->keyexpr, 
        topic
    );
    if (keyexpr_result != 0) {
        free(*publisher);
        *publisher = NULL;
        return ROS2_ZENOH_ERROR_ZENOH;
    }

    // Create Zenoh publisher
    z_publisher_options_t pub_options;
    z_publisher_options_default(&pub_options);
    
    z_result_t pub_result = z_declare_publisher(
        z_session_loan(&node->session),
        &(*publisher)->publisher,
        z_keyexpr(&(*publisher)->keyexpr),
        &pub_options
    );
    if (pub_result != 0) {
        z_drop(z_move(&(*publisher)->keyexpr));
        free(*publisher);
        *publisher = NULL;
        return ROS2_ZENOH_ERROR_ZENOH;
    }
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

    // Serialize message using Micro-CDR
    uint8_t buffer[1024];  // Fixed buffer size for simplicity
    ucdrBuffer ucdr_buffer;
    ucdr_init_buffer(&ucdr_buffer, buffer, sizeof(buffer));

    // Serialize based on message type
    bool success = false;
    if (strcmp(publisher->message_type, "geometry_msgs/msg/Twist") == 0) {
        success = ros2_twist_serialize((const ros2_twist_t*)message, &ucdr_buffer);
    } else if (strcmp(publisher->message_type, "geometry_msgs/msg/TwistStamped") == 0) {
        success = ros2_twist_stamped_serialize((const ros2_twist_stamped_t*)message, &ucdr_buffer);
    } else if (strcmp(publisher->message_type, "geometry_msgs/msg/Pose") == 0) {
        success = ros2_pose_serialize((const ros2_pose_t*)message, &ucdr_buffer);
    } else if (strcmp(publisher->message_type, "geometry_msgs/msg/PoseStamped") == 0) {
        success = ros2_pose_stamped_serialize((const ros2_pose_stamped_t*)message, &ucdr_buffer);
    } else if (strcmp(publisher->message_type, "geometry_msgs/msg/Vector3") == 0) {
        success = ros2_vector3_serialize((const ros2_vector3_t*)message, &ucdr_buffer);
    } else if (strcmp(publisher->message_type, "geometry_msgs/msg/Point") == 0) {
        success = ros2_point_serialize((const ros2_point_t*)message, &ucdr_buffer);
    } else if (strcmp(publisher->message_type, "geometry_msgs/msg/PointStamped") == 0) {
        success = ros2_point_stamped_serialize((const ros2_point_stamped_t*)message, &ucdr_buffer);
    }

    if (!success) {
        return ROS2_ZENOH_ERROR_SERIALIZATION;
    }

    // Publish via Zenoh
    size_t data_size = ucdr_buffer_length(&ucdr_buffer);
    z_publisher_put_options_t options;
    z_publisher_put_options_default(&options);
    
    z_encoding_t encoding;
    z_encoding_default(&encoding);
    options.encoding = encoding;
    
    z_bytes_t payload = {
        .start = buffer,
        .len = data_size
    };
    
    z_result_t result = z_publisher_put(
        z_publisher_loan(&publisher->publisher),
        z_move(&payload),
        &options
    );
    
    if (result != 0) {
        return ROS2_ZENOH_ERROR_ZENOH;
    }

    return ROS2_ZENOH_OK;
}

ros2_zenoh_ret_t ros2_zenoh_publisher_destroy(ros2_zenoh_publisher_t* publisher) {
    if (!publisher) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

#ifdef ZENOH_AVAILABLE
    z_drop(z_move(&publisher->publisher));
    z_drop(z_move(&publisher->keyexpr));
#endif
    free(publisher);

    return ROS2_ZENOH_OK;
}

// Subscriber callback wrapper
void ros2_zenoh_subscriber_callback(const z_sample_t* sample, void* arg) {
    ros2_zenoh_subscriber_t* subscriber = (ros2_zenoh_subscriber_t*)arg;
    
    if (!subscriber || !subscriber->callback) {
        return;
    }

    // Deserialize message using Micro-CDR
    ucdrBuffer ucdr_buffer;
    ucdr_init_buffer(&ucdr_buffer, (uint8_t*)sample->payload.start, sample->payload.len);

    // Allocate message buffer
    void* message = malloc(subscriber->message_size);
    if (!message) {
        return;
    }

    // Deserialize based on message type
    bool success = false;
    if (strcmp(subscriber->message_type, "geometry_msgs/msg/Twist") == 0) {
        success = ros2_twist_deserialize((ros2_twist_t*)message, &ucdr_buffer);
    } else if (strcmp(subscriber->message_type, "geometry_msgs/msg/TwistStamped") == 0) {
        success = ros2_twist_stamped_deserialize((ros2_twist_stamped_t*)message, &ucdr_buffer);
    } else if (strcmp(subscriber->message_type, "geometry_msgs/msg/Pose") == 0) {
        success = ros2_pose_deserialize((ros2_pose_t*)message, &ucdr_buffer);
    } else if (strcmp(subscriber->message_type, "geometry_msgs/msg/PoseStamped") == 0) {
        success = ros2_pose_stamped_deserialize((ros2_pose_stamped_t*)message, &ucdr_buffer);
    } else if (strcmp(subscriber->message_type, "geometry_msgs/msg/Vector3") == 0) {
        success = ros2_vector3_deserialize((ros2_vector3_t*)message, &ucdr_buffer);
    } else if (strcmp(subscriber->message_type, "geometry_msgs/msg/Point") == 0) {
        success = ros2_point_deserialize((ros2_point_t*)message, &ucdr_buffer);
    } else if (strcmp(subscriber->message_type, "geometry_msgs/msg/PointStamped") == 0) {
        success = ros2_point_stamped_deserialize((ros2_point_stamped_t*)message, &ucdr_buffer);
    }

    if (success) {
        subscriber->callback(message, subscriber->user_data);
    }

    free(message);
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

#ifdef ZENOH_AVAILABLE
    // Create Zenoh key expression
    z_result_t keyexpr_result = z_declare_keyexpr(
        z_session_loan(&node->session), 
        &(*subscriber)->keyexpr, 
        topic
    );
    if (keyexpr_result != 0) {
        free(*subscriber);
        *subscriber = NULL;
        return ROS2_ZENOH_ERROR_ZENOH;
    }

    // Create Zenoh subscriber
    z_subscriber_options_t sub_options;
    z_subscriber_options_default(&sub_options);
    
    z_closure_sample_t callback_closure = {
        .call = ros2_zenoh_subscriber_callback,
        .context = *subscriber
    };
    
    z_result_t sub_result = z_declare_subscriber(
        z_session_loan(&node->session),
        &(*subscriber)->subscriber,
        z_keyexpr(&(*subscriber)->keyexpr),
        z_move(&callback_closure),
        &sub_options
    );
    if (sub_result != 0) {
        z_drop(z_move(&(*subscriber)->keyexpr));
        free(*subscriber);
        *subscriber = NULL;
        return ROS2_ZENOH_ERROR_ZENOH;
    }
#endif

    return ROS2_ZENOH_OK;
}

ros2_zenoh_ret_t ros2_zenoh_subscriber_destroy(ros2_zenoh_subscriber_t* subscriber) {
    if (!subscriber) {
        return ROS2_ZENOH_ERROR_INVALID_ARGUMENT;
    }

#ifdef ZENOH_AVAILABLE
    z_drop(z_move(&subscriber->subscriber));
    z_drop(z_move(&subscriber->keyexpr));
#endif
    free(subscriber);

    return ROS2_ZENOH_OK;
}
