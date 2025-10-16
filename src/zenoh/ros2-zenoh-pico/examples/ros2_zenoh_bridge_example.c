#include "ros2_zenoh_pico/ros2_zenoh_bridge.h"
#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

// Simple Twist message structure (matches ROS2 geometry_msgs/Twist)
typedef struct {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
} twist_msg_t;

// Callback for receiving messages from Zenoh and republishing to ROS2
void zenoh_to_ros2_callback(const char* topic, const void* data, size_t data_size, void* user_data) {
    printf("DEBUG: zenoh_to_ros2_callback called!\n");
    fflush(stdout);
    
    // Deserialize the CDR-encoded Twist message
    twist_msg_t twist;
    if (ros2_zenoh_bridge_deserialize_twist((const uint8_t*)data, data_size, &twist) != ROS2_ZENOH_BRIDGE_OK) {
        printf("Failed to deserialize Twist message from Zenoh\n");
        fflush(stdout);
        return;
    }
    
    // Convert Zenoh topic back to ROS2 topic
    const char* ros2_topic = ros2_zenoh_bridge_zenoh_to_ros2_topic(topic, "ros2");
    if (!ros2_topic) {
        printf("Failed to convert Zenoh topic to ROS2 topic: %s\n", topic);
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
    
    printf(">> [Bridge] Received from Zenoh topic '%s' -> ROS2 topic '%s':\n", topic, ros2_topic);
    printf("   Linear:  x=%.3f, y=%.3f, z=%.3f (speed=%.3f m/s)\n", 
           twist.linear_x, twist.linear_y, twist.linear_z, linear_speed);
    printf("   Angular: x=%.3f, y=%.3f, z=%.3f (speed=%.3f rad/s)\n", 
           twist.angular_x, twist.angular_y, twist.angular_z, angular_speed);
    printf("   CDR Size: %zu bytes\n", data_size);
    printf("   Motion: %s\n", 
           linear_speed > 0.1f ? (angular_speed > 0.1f ? "Moving + Turning" : "Moving Forward") :
           (angular_speed > 0.1f ? "Turning in Place" : "Stopped"));
    printf("   [Bridge would republish to ROS2 topic: %s]\n", ros2_topic);
    printf("------------------------------------------------------------\n");
    fflush(stdout);
}

int main(int argc, char** argv) {
    printf("=== ROS2-Zenoh Bridge Example ===\n");
    fflush(stdout);
    
    // Bridge configuration
    ros2_zenoh_bridge_config_t bridge_config = ROS2_ZENOH_BRIDGE_DEFAULT_CONFIG;
    bridge_config.zenoh_endpoint = "tcp/172.18.0.2:7447";
    bridge_config.ros2_namespace = "/";
    bridge_config.zenoh_prefix = "ros2";
    bridge_config.verbose = true;

    // Create bridge
    ros2_zenoh_bridge_t* bridge = ros2_zenoh_bridge_create(&bridge_config);
    if (!bridge) {
        printf("Failed to create ROS2-Zenoh bridge\n");
        fflush(stdout);
        return 1;
    }

    printf("ROS2-Zenoh Bridge Example\n");
    printf("Bridging ROS2 Twist messages to/from Zenoh\n");
    fflush(stdout);

    // Add topic mapping for Twist messages
    const char* ros2_topic = "/turtle1/cmd_vel";
    const char* message_type = "geometry_msgs/msg/Twist";
    
    int result = ros2_zenoh_bridge_add_topic_mapping(
        bridge,
        ros2_topic,
        message_type,
        ROS2_ZENOH_BRIDGE_MODE_ZENOH_TO_ROS2
    );
    
    if (result != ROS2_ZENOH_BRIDGE_OK) {
        printf("Failed to add topic mapping: %s\n", ros2_zenoh_bridge_get_error_string(result));
        fflush(stdout);
        ros2_zenoh_bridge_destroy(bridge);
        return 1;
    }

    // Get the corresponding Zenoh topic
    const char* zenoh_topic = ros2_zenoh_bridge_ros2_to_zenoh_topic(ros2_topic, bridge_config.zenoh_prefix);
    printf("Topic mapping created:\n");
    printf("  ROS2 topic: %s\n", ros2_topic);
    printf("  Zenoh topic: %s\n", zenoh_topic);
    printf("  Message type: %s\n", message_type);
    fflush(stdout);

    // Create Zenoh subscriber to receive messages
    ros2_zenoh_pico_subscriber_config_t sub_config = ROS2_ZENOH_PICO_DEFAULT_SUBSCRIBER_CONFIG;
    sub_config.topic = zenoh_topic;
    sub_config.callback = zenoh_to_ros2_callback;
    sub_config.user_data = bridge;

    printf("Creating Zenoh subscriber for topic: %s\n", zenoh_topic);
    fflush(stdout);
    
    ros2_zenoh_pico_node_t* zenoh_node = (ros2_zenoh_pico_node_t*)ros2_zenoh_bridge_get_zenoh_node(bridge);
    ros2_zenoh_pico_node_t* sub_node = ros2_zenoh_pico_create_subscriber(zenoh_node, &sub_config);
    if (!sub_node) {
        printf("Failed to create Zenoh subscriber\n");
        fflush(stdout);
        ros2_zenoh_bridge_destroy(bridge);
        return 1;
    }

    // Start the bridge
    result = ros2_zenoh_bridge_start(bridge);
    if (result != ROS2_ZENOH_BRIDGE_OK) {
        printf("Failed to start bridge: %s\n", ros2_zenoh_bridge_get_error_string(result));
        fflush(stdout);
        ros2_zenoh_bridge_destroy(bridge);
        return 1;
    }

    printf("Bridge started successfully!\n");
    printf("Waiting for messages on Zenoh topic: %s\n", zenoh_topic);
    printf("Messages will be converted to ROS2 topic: %s\n", ros2_topic);
    printf("Press CTRL-C to quit...\n");
    fflush(stdout);

    // Keep running to receive and bridge messages
    int count = 0;
    while (true) {
        printf("DEBUG: Bridge running, iteration %d\n", count++);
        fflush(stdout);
        ros2_zenoh_bridge_sleep_ms(1000);
        
        // Exit after 30 iterations for testing
        if (count > 30) {
            printf("Bridge test completed after 30 iterations\n");
            break;
        }
    }

    // Cleanup
    ros2_zenoh_bridge_stop(bridge);
    ros2_zenoh_bridge_destroy(bridge);
    printf("ROS2-Zenoh bridge example completed\n");
    fflush(stdout);
    
    return 0;
}
