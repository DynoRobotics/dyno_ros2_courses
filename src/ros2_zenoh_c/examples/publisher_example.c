/**
 * @file publisher_example.c
 * @brief Example publisher using ros2_zenoh_c library
 */

#include "ros2_zenoh_c/ros2_zenoh_c.h"
#include "ros2_zenoh_c/ros2_messages.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

int main() {
    ros2_zenoh_ret_t ret;
    ros2_zenoh_node_t* node = NULL;
    ros2_zenoh_publisher_t* publisher = NULL;
    
    printf("🚀 Starting ROS 2 Zenoh C Publisher Example\n");
    
    // Initialize node
    ret = ros2_zenoh_node_init(&node, "zenoh_publisher_c");
    if (ret != ROS2_ZENOH_OK) {
        printf("❌ Failed to initialize node: %s\n", ros2_zenoh_get_error_string(ret));
        return 1;
    }
    printf("✅ Node initialized: %s\n", "zenoh_publisher_c");
    
    // Create publisher
    ret = ros2_zenoh_create_publisher(node, &publisher, 
                                     "/turtle1/cmd_vel", 
                                     "geometry_msgs/msg/Twist", 
                                     sizeof(ros2_twist_t));
    if (ret != ROS2_ZENOH_OK) {
        printf("❌ Failed to create publisher: %s\n", ros2_zenoh_get_error_string(ret));
        ros2_zenoh_node_destroy(node);
        return 1;
    }
    printf("✅ Publisher created for topic: %s\n", "/turtle1/cmd_vel");
    
    // Publish messages
    printf("📤 Publishing Twist messages for 10 seconds...\n");
    
    ros2_twist_t twist;
    time_t start_time = time(NULL);
    
    while (time(NULL) - start_time < 10) {
        // Set linear and angular velocities
        twist.linear.x = 1.0f;
        twist.linear.y = 0.0f;
        twist.linear.z = 0.0f;
        twist.angular.x = 0.0f;
        twist.angular.y = 0.0f;
        twist.angular.z = 0.5f;
        
        // Publish message
        ret = ros2_zenoh_publish(publisher, &twist);
        if (ret != ROS2_ZENOH_OK) {
            printf("❌ Failed to publish: %s\n", ros2_zenoh_get_error_string(ret));
            break;
        }
        
        printf("📤 Published: linear.x=%.1f, angular.z=%.1f\n", 
               twist.linear.x, twist.angular.z);
        
        sleep(1);
    }
    
    // Send stop command
    printf("🛑 Sending stop command...\n");
    twist.linear.x = 0.0f;
    twist.linear.y = 0.0f;
    twist.linear.z = 0.0f;
    twist.angular.x = 0.0f;
    twist.angular.y = 0.0f;
    twist.angular.z = 0.0f;
    
    ret = ros2_zenoh_publish(publisher, &twist);
    if (ret != ROS2_ZENOH_OK) {
        printf("❌ Failed to publish stop command: %s\n", ros2_zenoh_get_error_string(ret));
    }
    
    // Cleanup
    ros2_zenoh_publisher_destroy(publisher);
    ros2_zenoh_node_destroy(node);
    
    printf("✅ Publisher example completed\n");
    return 0;
}
