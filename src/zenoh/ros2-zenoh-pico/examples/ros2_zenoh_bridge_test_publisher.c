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

// CDR serialization function for Twist message
int serialize_twist(const twist_msg_t* twist, uint8_t* buffer, size_t buffer_size, size_t* serialized_size) {
#ifdef MICROCDR_AVAILABLE
    ucdrBuffer writer;
    ucdr_init_buffer(&writer, buffer, buffer_size);
    
    // Serialize linear velocity (Vector3)
    if (!ucdr_serialize_float(&writer, twist->linear_x) ||
        !ucdr_serialize_float(&writer, twist->linear_y) ||
        !ucdr_serialize_float(&writer, twist->linear_z)) {
        return -1;
    }
    
    // Serialize angular velocity (Vector3)
    if (!ucdr_serialize_float(&writer, twist->angular_x) ||
        !ucdr_serialize_float(&writer, twist->angular_y) ||
        !ucdr_serialize_float(&writer, twist->angular_z)) {
        return -1;
    }
    
    *serialized_size = ucdr_buffer_length(&writer);
    return 0;
#else
    return -1;
#endif
}

int main(int argc, char** argv) {
    printf("=== ROS2 Twist Publisher for Bridge Test ===\n");
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

    printf("ROS2 Twist Publisher for Bridge Test\n");
    printf("Publishing to Zenoh topic: ros2/turtle1/cmd_vel (ROS2 topic: /turtle1/cmd_vel)\n");
    fflush(stdout);

    // Create Twist message
    twist_msg_t twist;
    uint8_t cdr_buffer[1024];
    size_t serialized_size;
    
    // Publish messages
    for (int i = 0; i < 10; i++) {
        // Create a simple motion pattern
        float angle = (float)i * 0.5f; // Simple increment
        
        // Linear velocity (forward motion)
        twist.linear_x = 2.0f;  // Forward speed
        twist.linear_y = 0.0f;
        twist.linear_z = 0.0f;
        
        // Angular velocity (turning)
        twist.angular_x = 0.0f;
        twist.angular_y = 0.0f;
        twist.angular_z = 0.8f * sinf(angle);  // Oscillating turn rate
        
        // Serialize the message
        if (serialize_twist(&twist, cdr_buffer, sizeof(cdr_buffer), &serialized_size) != 0) {
            printf("Failed to serialize Twist message %d\n", i);
            fflush(stdout);
            continue;
        }
        
        // Publish the CDR-encoded message to the bridge topic
        int result = ros2_zenoh_pico_publish(
            node,
            "ros2/turtle1/cmd_vel",  // This matches what the bridge is listening to
            cdr_buffer,
            serialized_size
        );
        
        if (result != ROS2_ZENOH_PICO_OK) {
            printf("Failed to publish message %d: %s\n", i, ros2_zenoh_pico_get_error_string(result));
            fflush(stdout);
            break;
        }
        
        printf("Published Twist[%d] to bridge topic: linear=(%.2f,%.2f,%.2f) angular=(%.2f,%.2f,%.2f) [%zu bytes]\n", 
               i, 
               twist.linear_x, twist.linear_y, twist.linear_z,
               twist.angular_x, twist.angular_y, twist.angular_z,
               serialized_size);
        fflush(stdout);
        
        ros2_zenoh_pico_sleep_ms(1000);  // 1 second delay
    }

    // Cleanup
    ros2_zenoh_pico_node_destroy(node);
    printf("Bridge test publisher completed\n");
    fflush(stdout);
    
    return 0;
}
