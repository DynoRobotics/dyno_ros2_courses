/**
 * @file zenoh_publisher_example.c
 * @brief Example publisher using zenoh directly with microCDR serialization
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include "zenoh.h"
#include <ucdr/microcdr.h>

#define DEFAULT_KEYEXPR "turtle1/cmd_vel"
#define BUFFER_SIZE 256

// Simple Twist message structure (geometry_msgs/Twist)
typedef struct {
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;
} twist_msg_t;

// Serialize twist message using microCDR
size_t serialize_twist(const twist_msg_t* msg, uint8_t* buffer, size_t buffer_size) {
    ucdrBuffer writer;
    ucdr_init_buffer(&writer, buffer, buffer_size);
    
    // Serialize each field
    ucdr_serialize_double(&writer, msg->linear_x);
    ucdr_serialize_double(&writer, msg->linear_y);
    ucdr_serialize_double(&writer, msg->linear_z);
    ucdr_serialize_double(&writer, msg->angular_x);
    ucdr_serialize_double(&writer, msg->angular_y);
    ucdr_serialize_double(&writer, msg->angular_z);
    
    return ucdr_buffer_length(&writer);
}

int main(int argc, char** argv) {
    zc_init_log_from_env_or("error");
    
    printf("🚀 Starting Zenoh Publisher Example with microCDR\n");
    
    // Open zenoh session
    z_owned_config_t config = z_config_default();
    z_owned_session_t s = z_open(z_move(config));
    if (!z_session_check(&s)) {
        printf("❌ Unable to open session!\n");
        return 1;
    }
    printf("✅ Session opened\n");
    
    // Create publisher
    z_owned_publisher_t pub = z_declare_publisher(z_session_loan(&s), z_keyexpr(DEFAULT_KEYEXPR), NULL);
    if (!z_publisher_check(&pub)) {
        printf("❌ Unable to declare publisher for key expression!\n");
        z_close(z_session_move(&s));
        return 1;
    }
    printf("✅ Publisher declared for key expression: %s\n", DEFAULT_KEYEXPR);
    
    // Publish messages
    uint8_t buffer[BUFFER_SIZE];
    twist_msg_t twist_msg = {0};
    
    for (int i = 0; i < 10; i++) {
        // Create a simple twist message
        twist_msg.linear_x = 1.0 + (i * 0.1);
        twist_msg.angular_z = 0.5 + (i * 0.05);
        
        // Serialize the message
        size_t serialized_size = serialize_twist(&twist_msg, buffer, BUFFER_SIZE);
        
        // Publish
        z_publisher_put(z_publisher_loan(&pub), buffer, serialized_size, NULL);
        printf("📤 Published twist: linear_x=%.2f, angular_z=%.2f (size: %zu bytes)\n", 
               twist_msg.linear_x, twist_msg.angular_z, serialized_size);
        
        sleep(1);
    }
    
    // Cleanup
    z_undeclare_publisher(z_move(pub));
    z_close(z_session_move(&s));
    printf("✅ Publisher example completed\n");
    
    return 0;
}
