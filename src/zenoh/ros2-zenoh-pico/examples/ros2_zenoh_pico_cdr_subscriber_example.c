#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#ifdef MICROCDR_AVAILABLE
#include <ucdr/microcdr.h>
#endif

// Simple ROS2 Twist message structure (geometry_msgs/Twist)
typedef struct {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
} twist_msg_t;

// CDR deserialization function for Twist message
int deserialize_twist(const uint8_t* buffer, size_t buffer_size, twist_msg_t* twist) {
#ifdef MICROCDR_AVAILABLE
    ucdrBuffer reader;
    ucdr_init_buffer(&reader, (uint8_t*)buffer, buffer_size);
    
    // Deserialize linear velocity (Vector3)
    if (!ucdr_deserialize_float(&reader, &twist->linear_x) ||
        !ucdr_deserialize_float(&reader, &twist->linear_y) ||
        !ucdr_deserialize_float(&reader, &twist->linear_z)) {
        return -1;
    }
    
    // Deserialize angular velocity (Vector3)
    if (!ucdr_deserialize_float(&reader, &twist->angular_x) ||
        !ucdr_deserialize_float(&reader, &twist->angular_y) ||
        !ucdr_deserialize_float(&reader, &twist->angular_z)) {
        return -1;
    }
    
    return 0;
#else
    return -1;
#endif
}

// Subscription callback
void twist_callback(const char* topic, const void* data, size_t data_size, void* user_data) {
    printf("DEBUG: twist_callback called!\n");
    fflush(stdout);
    
    // Deserialize the CDR-encoded Twist message
    twist_msg_t twist;
    if (deserialize_twist((const uint8_t*)data, data_size, &twist) != 0) {
        printf("Failed to deserialize Twist message\n");
        fflush(stdout);
        return;
    }
    
    // Calculate motion characteristics
    float linear_speed = sqrtf(twist.linear_x * twist.linear_x + 
                              twist.linear_y * twist.linear_y + 
                              twist.linear_z * twist.linear_z);
    float angular_speed = sqrtf(twist.angular_x * twist.angular_x + 
                                twist.angular_y * twist.angular_y + 
                                twist.angular_z * twist.angular_z);
    
    printf(">> [CDR Subscriber] Received Twist on '%s':\n", topic);
    printf("   Linear:  x=%.3f, y=%.3f, z=%.3f (speed=%.3f m/s)\n", 
           twist.linear_x, twist.linear_y, twist.linear_z, linear_speed);
    printf("   Angular: x=%.3f, y=%.3f, z=%.3f (speed=%.3f rad/s)\n", 
           twist.angular_x, twist.angular_y, twist.angular_z, angular_speed);
    printf("   CDR Size: %zu bytes\n", data_size);
    printf("   Motion: %s\n", 
           linear_speed > 0.1f ? (angular_speed > 0.1f ? "Moving + Turning" : "Moving Forward") :
           (angular_speed > 0.1f ? "Turning in Place" : "Stopped"));
    printf("------------------------------------------------------------\n");
    fflush(stdout);
}

int main(int argc, char** argv) {
    printf("=== ROS2 Zenoh Pico CDR Subscriber Example ===\n");
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

    printf("ROS2 Zenoh Pico CDR Subscriber Example\n");
    printf("Subscribing to geometry_msgs/Twist messages on topic: demo/ros2_zenoh_pico/twist\n");
    fflush(stdout);

    // Create subscriber
    ros2_zenoh_pico_subscriber_config_t sub_config = ROS2_ZENOH_PICO_DEFAULT_SUBSCRIBER_CONFIG;
    sub_config.topic = "demo/ros2_zenoh_pico/twist";
    sub_config.callback = twist_callback;
    sub_config.user_data = NULL;

    printf("About to create CDR subscriber...\n");
    fflush(stdout);
    
    ros2_zenoh_pico_node_t* sub_node = ros2_zenoh_pico_create_subscriber(node, &sub_config);
    if (!sub_node) {
        printf("Failed to create subscriber\n");
        fflush(stdout);
        ros2_zenoh_pico_node_destroy(node);
        return 1;
    }

    printf("CDR Subscriber created successfully!\n");
    printf("Waiting for CDR-encoded Twist messages...\n");
    printf("Press CTRL-C to quit...\n");
    fflush(stdout);

    // Keep running to receive messages
    while (true) {
        ros2_zenoh_pico_sleep_ms(100);
    }

    // Cleanup
    ros2_zenoh_pico_node_destroy(node);
    printf("CDR Subscriber example completed\n");
    fflush(stdout);
    
    return 0;
}
