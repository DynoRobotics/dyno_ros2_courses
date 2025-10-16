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
    printf("=== Minimal ROS2 Zenoh Pico Subscriber Test ===\n");
    fflush(stdout);
    
    // Configuration
    ros2_zenoh_pico_config_t config = ROS2_ZENOH_PICO_DEFAULT_CONFIG;
    config.endpoint = "tcp/172.18.0.2:7447";
    config.verbose = true;

    // Create node
    ros2_zenoh_pico_node_t* node = ros2_zenoh_pico_node_create(&config);
    if (!node) {
        printf("Failed to create zenoh node\n");
        fflush(stdout);
        return 1;
    }

    printf("Node created successfully\n");
    fflush(stdout);

    // Create subscriber
    ros2_zenoh_pico_subscriber_config_t sub_config = ROS2_ZENOH_PICO_DEFAULT_SUBSCRIBER_CONFIG;
    sub_config.topic = "demo/ros2_zenoh_pico/**";
    sub_config.callback = subscription_callback;
    sub_config.user_data = NULL;

    printf("Creating subscriber...\n");
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

    printf("Waiting for messages...\n");
    fflush(stdout);

    // Keep running to receive messages
    int count = 0;
    while (count < 20) {
        printf("DEBUG: Main loop iteration %d\n", count++);
        fflush(stdout);
        ros2_zenoh_pico_sleep_ms(1000);
    }

    // Cleanup
    ros2_zenoh_pico_node_destroy(node);
    printf("Minimal subscriber test completed\n");
    fflush(stdout);
    
    return 0;
}
