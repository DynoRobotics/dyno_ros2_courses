#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include "zenoh-pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Internal publisher data structure
typedef struct publisher_entry_t {
    z_owned_publisher_t publisher;
    char* topic;
    struct publisher_entry_t* next;
} publisher_entry_t;

// Internal node structure
struct ros2_zenoh_pico_node_t {
    z_owned_session_t session;
    bool is_valid;
    ros2_zenoh_pico_config_t config;
    publisher_entry_t* publishers;  // Linked list of publishers
};

// Publisher implementation
ros2_zenoh_pico_node_t* ros2_zenoh_pico_create_publisher(
    ros2_zenoh_pico_node_t* node,
    const ros2_zenoh_pico_publisher_config_t* config
) {
    if (!node || !config || !config->topic) {
        return NULL;
    }

    if (!ros2_zenoh_pico_node_is_valid(node)) {
        return NULL;
    }

    // Check if publisher already exists for this topic
    publisher_entry_t* current = node->publishers;
    while (current) {
        if (strcmp(current->topic, config->topic) == 0) {
            if (node->config.verbose) {
                printf("Publisher already exists for topic: %s\n", config->topic);
            }
            return node;
        }
        current = current->next;
    }

    // Create keyexpr view
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, config->topic) < 0) {
        if (node->config.verbose) {
            printf("Invalid key expression: %s\n", config->topic);
        }
        return NULL;
    }

    // Create publisher
    z_owned_publisher_t publisher;
    if (z_declare_publisher(z_session_loan(&node->session), &publisher, z_view_keyexpr_loan(&ke), NULL) < 0) {
        if (node->config.verbose) {
            printf("Unable to declare publisher for topic: %s\n", config->topic);
        }
        return NULL;
    }

    // Create publisher entry
    publisher_entry_t* entry = malloc(sizeof(publisher_entry_t));
    if (!entry) {
        z_publisher_drop(z_publisher_move(&publisher));
        return NULL;
    }

    entry->publisher = publisher;
    entry->topic = strdup(config->topic);
    entry->next = node->publishers;
    node->publishers = entry;

    if (node->config.verbose) {
        printf("Publisher created for topic: %s\n", config->topic);
    }

    return node;
}

int ros2_zenoh_pico_publish(
    ros2_zenoh_pico_node_t* node,
    const char* topic,
    const void* data,
    size_t data_size
) {
    if (!node || !topic || !data || data_size == 0) {
        return ROS2_ZENOH_PICO_ERROR_INVALID_ARGUMENT;
    }

    if (!ros2_zenoh_pico_node_is_valid(node)) {
        return ROS2_ZENOH_PICO_ERROR_SESSION_FAILED;
    }

    // Find existing publisher for this topic
    publisher_entry_t* current = node->publishers;
    while (current) {
        if (strcmp(current->topic, topic) == 0) {
            // Found existing publisher, use it
            break;
        }
        current = current->next;
    }

    // If no publisher found, create one
    if (!current) {
        ros2_zenoh_pico_publisher_config_t pub_config = ROS2_ZENOH_PICO_DEFAULT_PUBLISHER_CONFIG;
        pub_config.topic = topic;
        
        if (!ros2_zenoh_pico_create_publisher(node, &pub_config)) {
            return ROS2_ZENOH_PICO_ERROR_PUBLISHER_FAILED;
        }
        
        // Find the newly created publisher
        current = node->publishers;
        while (current && strcmp(current->topic, topic) != 0) {
            current = current->next;
        }
        
        if (!current) {
            return ROS2_ZENOH_PICO_ERROR_PUBLISHER_FAILED;
        }
    }

    // Create payload from binary data
    z_owned_bytes_t payload;
    if (z_bytes_copy_from_buf(&payload, data, data_size) < 0) {
        if (node->config.verbose) {
            printf("Failed to create payload from binary data\n");
        }
        return ROS2_ZENOH_PICO_ERROR_PUBLISHER_FAILED;
    }

    // Publish using existing publisher
    if (z_publisher_put(z_publisher_loan(&current->publisher), z_bytes_move(&payload), NULL) < 0) {
        if (node->config.verbose) {
            printf("Failed to publish to topic: %s\n", topic);
        }
        return ROS2_ZENOH_PICO_ERROR_PUBLISHER_FAILED;
    }

    if (node->config.verbose) {
        printf("Published %zu bytes to topic: %s\n", data_size, topic);
    }

    return ROS2_ZENOH_PICO_OK;
}

int ros2_zenoh_pico_publish_with_attachment(
    ros2_zenoh_pico_node_t* node,
    const char* topic,
    const void* data,
    size_t data_size,
    const void* attachment,
    size_t attachment_size
) {
    if (!node || !topic || !data || data_size == 0) {
        return ROS2_ZENOH_PICO_ERROR_INVALID_ARGUMENT;
    }

    // Find existing publisher for this topic
    publisher_entry_t* current = node->publishers;
    printf("DEBUG: Looking for publisher for topic: %s\n", topic);
    fflush(stdout);
    
    while (current) {
        printf("DEBUG: Checking publisher topic: %s\n", current->topic);
        fflush(stdout);
        if (strcmp(current->topic, topic) == 0) {
            printf("DEBUG: Found matching publisher\n");
            fflush(stdout);
            break;
        }
        current = current->next;
    }
    
    if (!current) {
        printf("DEBUG: No publisher found for topic: %s\n", topic);
        fflush(stdout);
        return ROS2_ZENOH_PICO_ERROR_PUBLISHER_FAILED;
    }

    // Create payload from binary data
    z_owned_bytes_t payload;
    if (z_bytes_copy_from_buf(&payload, data, data_size) < 0) {
        if (node->config.verbose) {
            printf("Failed to create payload from binary data\n");
        }
        return ROS2_ZENOH_PICO_ERROR_PUBLISHER_FAILED;
    }

    // Create attachment if provided
    z_owned_bytes_t attachment_bytes = {0};
    if (attachment && attachment_size > 0) {
        if (z_bytes_copy_from_buf(&attachment_bytes, attachment, attachment_size) < 0) {
            if (node->config.verbose) {
                printf("Failed to create attachment from binary data\n");
            }
            return ROS2_ZENOH_PICO_ERROR_PUBLISHER_FAILED;
        }
    }

    // Set up publisher options with encoding and optional attachment
    z_publisher_put_options_t options;
    z_publisher_put_options_default(&options);

    // Set encoding to application/x-cdr so rmw_zenoh can parse the payload
    z_owned_encoding_t encoding;
    if (z_encoding_from_str(&encoding, "application/x-cdr") == Z_OK) {
        options.encoding = z_encoding_move(&encoding);
    }

    if (attachment && attachment_size > 0) {
        options.attachment = z_bytes_move(&attachment_bytes);
    }

    // Publish using existing publisher with attachment
    printf("DEBUG: About to call z_publisher_put with attachment\n");
    fflush(stdout);
    
    if (z_publisher_put(z_publisher_loan(&current->publisher), z_bytes_move(&payload), &options) < 0) {
        if (node->config.verbose) {
            printf("Failed to publish to topic: %s\n", topic);
        }
        return ROS2_ZENOH_PICO_ERROR_PUBLISHER_FAILED;
    }
    
    printf("DEBUG: z_publisher_put succeeded\n");
    fflush(stdout);

    if (node->config.verbose) {
        printf("Published %zu bytes to topic: %s (attachment: %zu bytes)\n", data_size, topic, attachment_size);
    }

    return ROS2_ZENOH_PICO_OK;
}