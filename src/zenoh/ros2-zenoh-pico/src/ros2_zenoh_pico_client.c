#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include "zenoh-pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Internal node structure
struct ros2_zenoh_pico_node_t {
    z_owned_session_t session;
    bool is_valid;
    ros2_zenoh_pico_config_t config;
};

// Client implementation (simplified stub)
ros2_zenoh_pico_node_t* ros2_zenoh_pico_create_client(
    ros2_zenoh_pico_node_t* node,
    const ros2_zenoh_pico_client_config_t* config
) {
    if (!node || !config || !config->keyexpr || !config->callback) {
        return NULL;
    }

    if (!ros2_zenoh_pico_node_is_valid(node)) {
        return NULL;
    }

    if (node->config.verbose) {
        printf("Client created for keyexpr: %s\n", config->keyexpr);
    }

    return node;
}

int ros2_zenoh_pico_request(
    ros2_zenoh_pico_node_t* node,
    const char* keyexpr,
    const void* data,
    size_t data_size
) {
    if (!node || !keyexpr || !data || data_size == 0) {
        return ROS2_ZENOH_PICO_ERROR_INVALID_ARGUMENT;
    }

    if (!ros2_zenoh_pico_node_is_valid(node)) {
        return ROS2_ZENOH_PICO_ERROR_SESSION_FAILED;
    }

    if (node->config.verbose) {
        printf("Request sent to keyexpr: %s\n", keyexpr);
    }

    return ROS2_ZENOH_PICO_OK;
}