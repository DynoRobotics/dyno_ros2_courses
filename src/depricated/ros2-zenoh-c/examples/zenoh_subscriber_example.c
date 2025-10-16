/**
 * @file zenoh_subscriber_example.c
 * @brief Example subscriber using zenoh directly with microCDR deserialization
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
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

// Deserialize twist message using microCDR
bool deserialize_twist(const uint8_t* buffer, size_t buffer_size, twist_msg_t* msg) {
    ucdrBuffer reader;
    ucdr_init_buffer(&reader, (uint8_t*)buffer, buffer_size);
    
    // Deserialize each field
    bool success = true;
    success &= ucdr_deserialize_double(&reader, &msg->linear_x);
    success &= ucdr_deserialize_double(&reader, &msg->linear_y);
    success &= ucdr_deserialize_double(&reader, &msg->linear_z);
    success &= ucdr_deserialize_double(&reader, &msg->angular_x);
    success &= ucdr_deserialize_double(&reader, &msg->angular_y);
    success &= ucdr_deserialize_double(&reader, &msg->angular_z);
    
    return success;
}

void data_handler(const z_sample_t* sample, void* arg) {
    (void)arg;
    
    printf("📥 Received data on key expression: %.*s\n", 
           (int)sample->keyexpr.len, sample->keyexpr.start);
    printf("   Payload size: %zu bytes\n", sample->payload.len);
    
    // Deserialize the twist message
    twist_msg_t twist_msg;
    if (deserialize_twist(sample->payload.start, sample->payload.len, &twist_msg)) {
        printf("   Twist: linear_x=%.2f, linear_y=%.2f, linear_z=%.2f\n",
               twist_msg.linear_x, twist_msg.linear_y, twist_msg.linear_z);
        printf("          angular_x=%.2f, angular_y=%.2f, angular_z=%.2f\n",
               twist_msg.angular_x, twist_msg.angular_y, twist_msg.angular_z);
    } else {
        printf("   ❌ Failed to deserialize twist message\n");
    }
    printf("\n");
}

int main(int argc, char** argv) {
    zc_init_log_from_env_or("error");
    
    printf("🚀 Starting Zenoh Subscriber Example with microCDR\n");
    
    // Open zenoh session
    z_owned_config_t config = z_config_default();
    z_owned_session_t s = z_open(z_move(config));
    if (!z_session_check(&s)) {
        printf("❌ Unable to open session!\n");
        return 1;
    }
    printf("✅ Session opened\n");
    
    // Create subscriber
    z_owned_closure_sample_t callback = z_closure(data_handler);
    z_owned_subscriber_t sub = z_declare_subscriber(z_session_loan(&s), 
                                                   z_keyexpr(DEFAULT_KEYEXPR), 
                                                   z_move(callback), 
                                                   NULL);
    if (!z_subscriber_check(&sub)) {
        printf("❌ Unable to declare subscriber for key expression!\n");
        z_close(z_session_move(&s));
        return 1;
    }
    printf("✅ Subscriber declared for key expression: %s\n", DEFAULT_KEYEXPR);
    printf("📡 Listening for messages... (Press Ctrl+C to stop)\n\n");
    
    // Keep the session alive
    while (1) {
        sleep(1);
    }
    
    // Cleanup (this won't be reached due to the infinite loop)
    z_undeclare_subscriber(z_move(sub));
    z_close(z_session_move(&s));
    
    return 0;
}
