/**
 * @file working_zenoh_example.c
 * @brief Working Zenoh pub/sub example using same session
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include "zenoh.h"

#define DEFAULT_KEYEXPR "turtle1/cmd_vel"
#define DEFAULT_VALUE "Twist message from C!"

static z_owned_session_t g_session;
static int g_messages_received = 0;
static int g_running = 1;

void data_handler(z_loaned_sample_t* sample, void* arg) {
    printf("🔔 CALLBACK TRIGGERED! Message #%d\n", ++g_messages_received);
    
    z_view_string_t key_string;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &key_string);

    z_owned_string_t payload_string;
    z_bytes_to_string(z_sample_payload(sample), &payload_string);

    printf("📨 [Subscriber] Received %s ('%.*s': '%.*s')\n", 
           "PUT",
           (int)z_string_len(z_loan(key_string)), z_string_data(z_loan(key_string)),
           (int)z_string_len(z_loan(payload_string)), z_string_data(z_loan(payload_string)));
    
    z_drop(z_move(payload_string));
}

void* publisher_thread(void* arg) {
    printf("📤 Publisher thread starting...\n");
    
    // Create key expression
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, DEFAULT_KEYEXPR) < 0) {
        printf("❌ Invalid key expression!\n");
        return NULL;
    }

    // Declare publisher
    z_owned_publisher_t pub;
    if (z_declare_publisher(z_loan(g_session), &pub, z_loan(ke), NULL) < 0) {
        printf("❌ Failed to declare publisher!\n");
        return NULL;
    }
    printf("✅ Publisher declared!\n");

    // Publish messages
    printf("📤 Publishing Twist messages for 10 seconds...\n");
    for (int i = 0; i < 10 && g_running; i++) {
        char msg[256];
        sprintf(msg, "[%ld] %s", time(NULL), DEFAULT_VALUE);
        
        z_publisher_put_options_t options;
        z_publisher_put_options_default(&options);

        z_owned_bytes_t payload;
        z_bytes_copy_from_str(&payload, msg);
        
        printf("📤 Publishing: '%s'\n", msg);
        z_publisher_put(z_loan(pub), z_move(payload), &options);
        
        sleep(1);
    }

    // Send stop command
    printf("🛑 Sending stop command...\n");
    z_publisher_put_options_t options;
    z_publisher_put_options_default(&options);
    
    z_owned_bytes_t payload;
    z_bytes_copy_from_str(&payload, "STOP");
    
    z_publisher_put(z_loan(pub), z_move(payload), &options);

    // Cleanup
    z_drop(z_move(pub));
    printf("📤 Publisher thread finished!\n");
    return NULL;
}

int main(int argc, char** argv) {
    printf("🚀 Working ROS 2 Zenoh C Pub/Sub Example\n");
    printf("=========================================\n");

    zc_init_log_from_env_or("error");

    // Parse arguments
    char* keyexpr = DEFAULT_KEYEXPR;
    char* value = DEFAULT_VALUE;
    
    if (argc > 1) keyexpr = argv[1];
    if (argc > 2) value = argv[2];

    // Open session
    printf("Opening session...\n");
    z_owned_config_t config;
    z_config_default(&config);
    
    if (z_open(&g_session, z_move(config), NULL) < 0) {
        printf("❌ Failed to open session!\n");
        return -1;
    }
    printf("✅ Session opened!\n");

    // Create key expression for subscriber
    printf("Creating key expression...\n");
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        printf("❌ Invalid key expression!\n");
        z_drop(z_move(g_session));
        return -1;
    }
    printf("✅ Key expression created!\n");

    // Create callback
    printf("Creating callback...\n");
    z_owned_closure_sample_t callback;
    z_closure(&callback, data_handler, NULL, NULL);
    printf("✅ Callback created!\n");

    // Declare subscriber
    printf("Declaring subscriber...\n");
    z_owned_subscriber_t sub;
    if (z_declare_subscriber(z_loan(g_session), &sub, z_loan(ke), z_move(callback), NULL) < 0) {
        printf("❌ Failed to declare subscriber!\n");
        z_drop(z_move(g_session));
        return -1;
    }
    printf("✅ Subscriber declared!\n");

    // Start publisher thread
    printf("Starting publisher thread...\n");
    pthread_t pub_thread;
    if (pthread_create(&pub_thread, NULL, publisher_thread, NULL) != 0) {
        printf("❌ Failed to create publisher thread!\n");
        z_drop(z_move(sub));
        z_drop(z_move(g_session));
        return -1;
    }

    // Wait for messages
    printf("📥 Listening for messages on topic '%s'...\n", keyexpr);
    printf("Press Ctrl+C to stop early\n");
    sleep(12); // Wait for publisher to finish + buffer

    // Stop publisher thread
    g_running = 0;
    pthread_join(pub_thread, NULL);

    // Cleanup
    printf("Cleaning up...\n");
    z_drop(z_move(sub));
    z_drop(z_move(g_session));
    
    printf("✅ Example completed! Received %d messages.\n", g_messages_received);
    return 0;
}
