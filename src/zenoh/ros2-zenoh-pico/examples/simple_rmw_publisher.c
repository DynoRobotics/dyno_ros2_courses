#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char** argv) {
    printf("=== Simple ROS2 Publisher (DDS Interop Key) ===\n");
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

    printf("Simple ROS2 Publisher\n");
    printf("Publishing to DDS interop key: 0/turtle1/cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a\n");
    fflush(stdout);

    // Publish messages using DDS interop key
    for (int i = 0; i < 10; i++) {
        char message[256];
        snprintf(message, sizeof(message), "Twist message %d: linear=(1.5,0,0) angular=(0,0,0.5)", i);
        
        // Use the DDS interop key format
        const char* dds_key = "0/turtle1/cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a";
        
        int result = ros2_zenoh_pico_publish(node, dds_key, message, strlen(message));
        
        if (result != ROS2_ZENOH_PICO_OK) {
            printf("Failed to publish message %d: %s\n", i, ros2_zenoh_pico_get_error_string(result));
            fflush(stdout);
            break;
        }
        
        printf("Published: %s\n", message);
        fflush(stdout);
        ros2_zenoh_pico_sleep_ms(1000);  // 1 second delay
    }

    // Cleanup
    ros2_zenoh_pico_node_destroy(node);
    printf("Simple ROS2 Publisher completed\n");
    fflush(stdout);
    
    return 0;
}
