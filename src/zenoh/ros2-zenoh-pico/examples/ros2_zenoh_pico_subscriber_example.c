#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Subscription callback
void subscription_callback(const char* topic, const void* data, size_t data_size, void* user_data) {
    printf("DEBUG: subscription_callback called!\n");
    fflush(stdout);
    printf(">> [Subscriber] Received ('%s': '%.*s')\n", topic, (int)data_size, (const char*)data);
    fflush(stdout);
}

int main(int argc, char** argv) {
    printf("=== ROS2 Zenoh Pico Subscriber Starting ===\n");
    fflush(stdout);
    
    // Configuration
    ros2_zenoh_pico_config_t config = ROS2_ZENOH_PICO_DEFAULT_CONFIG;
           config.endpoint = "tcp/172.18.0.2:7447";  // Your Zenoh router
    config.verbose = true;

    // Create node
    ros2_zenoh_pico_node_t* node = ros2_zenoh_pico_node_create(&config);
    if (!node) {
        printf("Failed to create zenoh node\n");
        fflush(stdout);
        return 1;
    }

    printf("ROS2 Zenoh Pico Subscriber Example\n");
    printf("Subscribing to topic: demo/ros2_zenoh_pico/**\n");
    fflush(stdout);

    // Create subscriber
    ros2_zenoh_pico_subscriber_config_t sub_config = ROS2_ZENOH_PICO_DEFAULT_SUBSCRIBER_CONFIG;
    sub_config.topic = "demo/ros2_zenoh_pico/**";
    sub_config.callback = subscription_callback;
    sub_config.user_data = NULL;

    printf("About to create subscriber...\n");
    fflush(stdout);
    
    ros2_zenoh_pico_node_t* sub_node = ros2_zenoh_pico_create_subscriber(node, &sub_config);
    if (!sub_node) {
        printf("Failed to create subscriber\n");
        fflush(stdout);
        ros2_zenoh_pico_node_destroy(node);
        return 1;
    }

    printf("Subscriber created successfully!\n");
    fflush(stdout);

    printf("Subscriber created. Waiting for messages...\n");
    printf("Press CTRL-C to quit...\n");
    fflush(stdout);

    // Keep running to receive messages
    while (true) {
        ros2_zenoh_pico_sleep_ms(100);
    }

    // Cleanup
    ros2_zenoh_pico_node_destroy(node);
    printf("Subscriber example completed\n");
    
    return 0;
}
