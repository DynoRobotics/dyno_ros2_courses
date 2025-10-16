#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char** argv) {
    // Configuration
    ros2_zenoh_pico_config_t config = ROS2_ZENOH_PICO_DEFAULT_CONFIG;
           config.endpoint = "tcp/172.18.0.2:7447";  // Your Zenoh router
    config.verbose = true;

    // Create node
    ros2_zenoh_pico_node_t* node = ros2_zenoh_pico_node_create(&config);
    if (!node) {
        printf("Failed to create zenoh node\n");
        return 1;
    }

    printf("ROS2 Zenoh Pico Publisher Example\n");
    printf("Publishing to topic: demo/ros2_zenoh_pico/pub\n");
    fflush(stdout);

    // Publish messages
    for (int i = 0; i < 100; i++) {
        char message[256];
        snprintf(message, sizeof(message), "[%3d] Hello from ROS2 Zenoh Pico Publisher!", i);
        
        int result = ros2_zenoh_pico_publish(
            node,
            "demo/ros2_zenoh_pico/pub",
            message,
            strlen(message)
        );
        
        if (result != ROS2_ZENOH_PICO_OK) {
            printf("Failed to publish message %d: %s\n", i, ros2_zenoh_pico_get_error_string(result));
            break;
        }
        
        printf("Published: %s\n", message);
        fflush(stdout);
        ros2_zenoh_pico_sleep_ms(1000);  // 1 second delay
    }

    // Cleanup
    ros2_zenoh_pico_node_destroy(node);
    printf("Publisher example completed\n");
    
    return 0;
}
