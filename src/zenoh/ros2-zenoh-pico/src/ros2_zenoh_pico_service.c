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

// Service implementation (simplified stub)
ros2_zenoh_pico_node_t* ros2_zenoh_pico_create_service(
    ros2_zenoh_pico_node_t* node,
    const ros2_zenoh_pico_service_config_t* config
) {
    if (!node || !config || !config->keyexpr || !config->callback) {
        return NULL;
    }

    if (!ros2_zenoh_pico_node_is_valid(node)) {
        return NULL;
    }

    if (node->config.verbose) {
        printf("Service created for keyexpr: %s\n", config->keyexpr);
    }

    return node;
}