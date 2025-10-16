#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include "zenoh-pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Internal subscriber data structure
typedef struct subscriber_entry_t {
    z_owned_subscriber_t subscriber;
    void* callback_data;
    struct subscriber_entry_t* next;
} subscriber_entry_t;

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
    subscriber_entry_t* subscribers;  // Linked list of subscribers
    publisher_entry_t* publishers;     // Linked list of publishers
};

// Error string mapping
static const char* error_strings[] = {
    "OK",
    "Invalid argument",
    "Initialization failed", 
    "Session failed",
    "Publisher failed",
    "Subscriber failed",
    "Client failed",
    "Service failed",
    "Memory allocation failed",
    "Timeout",
    "Network error"
};

const char* ros2_zenoh_pico_get_error_string(ros2_zenoh_pico_error_t error) {
    int index = -(int)error;
    if (index >= 0 && index < sizeof(error_strings) / sizeof(error_strings[0])) {
        return error_strings[index];
    }
    return "Unknown error";
}

ros2_zenoh_pico_node_t* ros2_zenoh_pico_node_create(const ros2_zenoh_pico_config_t* config) {
    if (!config) {
        printf("DEBUG: Invalid config for node creation\n");
        return NULL;
    }

    printf("DEBUG: Creating zenoh node with endpoint: %s\n", config->endpoint ? config->endpoint : "default");

    ros2_zenoh_pico_node_t* node = malloc(sizeof(ros2_zenoh_pico_node_t));
    if (!node) {
        printf("DEBUG: Failed to allocate memory for node\n");
        return NULL;
    }

    // Initialize node
    memset(node, 0, sizeof(ros2_zenoh_pico_node_t));
    node->config = *config;
    node->is_valid = false;
    node->subscribers = NULL; // Initialize subscriber list
    node->publishers = NULL;   // Initialize publisher list

    // Configure zenoh-pico
    z_owned_config_t zenoh_config;
    z_config_default(&zenoh_config);
    
    if (config->endpoint) {
        zp_config_insert(z_config_loan_mut(&zenoh_config), Z_CONFIG_CONNECT_KEY, config->endpoint);
    }

    if (config->mode) {
        zp_config_insert(z_config_loan_mut(&zenoh_config), Z_CONFIG_MODE_KEY, config->mode);
    }

    // Open session
    if (z_open(&node->session, z_config_move(&zenoh_config), NULL) < 0) {
        if (config->verbose) {
            printf("Failed to open zenoh session\n");
        }
        free(node);
        return NULL;
    }

    // Start read and lease tasks for zenoh-pico
    if (zp_start_read_task(z_session_loan_mut(&node->session), NULL) < 0 || 
        zp_start_lease_task(z_session_loan_mut(&node->session), NULL) < 0) {
        if (config->verbose) {
            printf("Unable to start read and lease tasks\n");
        }
        z_session_drop(z_session_move(&node->session));
        free(node);
        return NULL;
    }

    node->is_valid = true;

    if (config->verbose) {
        printf("Zenoh session opened successfully\n");
    }

    return node;
}

int ros2_zenoh_pico_node_destroy(ros2_zenoh_pico_node_t* node) {
    if (!node) {
        return ROS2_ZENOH_PICO_ERROR_INVALID_ARGUMENT;
    }

    // Clean up subscribers
    subscriber_entry_t* current = node->subscribers;
    while (current) {
        subscriber_entry_t* next = current->next;
        z_subscriber_drop(z_subscriber_move(&current->subscriber));
        free(current->callback_data);
        free(current);
        current = next;
    }

    // Clean up publishers
    publisher_entry_t* pub_current = node->publishers;
    while (pub_current) {
        publisher_entry_t* pub_next = pub_current->next;
        z_publisher_drop(z_publisher_move(&pub_current->publisher));
        free(pub_current->topic);
        free(pub_current);
        pub_current = pub_next;
    }

    if (node->is_valid) {
        z_session_drop(z_session_move(&node->session));
    }

    free(node);
    return ROS2_ZENOH_PICO_OK;
}

bool ros2_zenoh_pico_node_is_valid(const ros2_zenoh_pico_node_t* node) {
    return node && node->is_valid;
}

int ros2_zenoh_pico_sleep_ms(uint32_t milliseconds) {
    usleep(milliseconds * 1000);
    return ROS2_ZENOH_PICO_OK;
}

void* ros2_zenoh_pico_node_get_session(ros2_zenoh_pico_node_t* node) {
    if (!node || !node->is_valid) {
        return NULL;
    }
    return &node->session;
}