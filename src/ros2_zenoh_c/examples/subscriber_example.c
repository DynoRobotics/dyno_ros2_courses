/**
 * @file subscriber_example.c
 * @brief Example subscriber using ros2_zenoh_c library
 */

#include "ros2_zenoh_c/ros2_zenoh_c.h"
#include "ros2_zenoh_c/ros2_messages.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

// Callback function for received messages
void twist_callback(const void* message, void* user_data) {
    const ros2_twist_t* twist = (const ros2_twist_t*)message;
    printf("📨 Received Twist message:\n");
    printf("   Linear:  x=%.2f, y=%.2f, z=%.2f\n", 
           twist->linear.x, twist->linear.y, twist->linear.z);
    printf("   Angular: x=%.2f, y=%.2f, z=%.2f\n", 
           twist->angular.x, twist->angular.y, twist->angular.z);
    printf("----------------------------------------\n");
}

int main() {
    ros2_zenoh_ret_t ret;
    ros2_zenoh_node_t* node = NULL;
    ros2_zenoh_subscriber_t* subscriber = NULL;
    
    printf("🚀 Starting ROS 2 Zenoh C Subscriber Example\n");
    
    // Initialize node
    ret = ros2_zenoh_node_init(&node, "zenoh_subscriber_c");
    if (ret != ROS2_ZENOH_OK) {
        printf("❌ Failed to initialize node: %s\n", ros2_zenoh_get_error_string(ret));
        return 1;
    }
    printf("✅ Node initialized: %s\n", "zenoh_subscriber_c");
    
    // Create subscriber
    ret = ros2_zenoh_create_subscriber(node, &subscriber, 
                                      "/turtle1/cmd_vel", 
                                      "geometry_msgs/msg/Twist", 
                                      sizeof(ros2_twist_t),
                                      twist_callback,
                                      NULL);
    if (ret != ROS2_ZENOH_OK) {
        printf("❌ Failed to create subscriber: %s\n", ros2_zenoh_get_error_string(ret));
        ros2_zenoh_node_destroy(node);
        return 1;
    }
    printf("✅ Subscriber created for topic: %s\n", "/turtle1/cmd_vel");
    
    // Listen for messages
    printf("📥 Listening for messages on topic '%s' for 30 seconds...\n", "/turtle1/cmd_vel");
    printf("Press Ctrl+C to stop early\n");
    
    sleep(30);
    
    // Cleanup
    ros2_zenoh_subscriber_destroy(subscriber);
    ros2_zenoh_node_destroy(node);
    
    printf("✅ Subscriber example completed\n");
    return 0;
}
