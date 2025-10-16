/**
 * @file subscriber_example_zenoh.c
 * @brief Example subscriber using direct Zenoh C API
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "zenoh.h"

#define DEFAULT_KEYEXPR "turtle1/cmd_vel"

const char* kind_to_str(z_sample_kind_t kind);

void data_handler(z_loaned_sample_t* sample, void* arg) {
    printf("🔔 Callback triggered!\n");
    
    z_view_string_t key_string;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &key_string);

    z_owned_string_t payload_string;
    z_bytes_to_string(z_sample_payload(sample), &payload_string);

    printf("📨 [Subscriber] Received %s ('%.*s': '%.*s')\n", 
           kind_to_str(z_sample_kind(sample)),
           (int)z_string_len(z_loan(key_string)), z_string_data(z_loan(key_string)),
           (int)z_string_len(z_loan(payload_string)), z_string_data(z_loan(payload_string)));

    const z_loaned_bytes_t* attachment = z_sample_attachment(sample);
    if (attachment != NULL) {
        z_owned_string_t attachment_string;
        z_bytes_to_string(attachment, &attachment_string);
        printf("   Attachment: %.*s\n", 
               (int)z_string_len(z_loan(attachment_string)), 
               z_string_data(z_loan(attachment_string)));
        z_drop(z_move(attachment_string));
    }
    
    z_drop(z_move(payload_string));
}

int main(int argc, char** argv) {
    zc_init_log_from_env_or("error");

    printf("🚀 Starting ROS 2 Zenoh C Subscriber Example\n");

    // Parse arguments
    char* keyexpr = DEFAULT_KEYEXPR;
    if (argc > 1) keyexpr = argv[1];

    // Open session
    printf("Opening session...\n");
    z_owned_config_t config;
    z_config_default(&config);
    
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str(&ke, keyexpr);

    z_owned_session_t s;
    if (z_open(&s, z_move(config), NULL) < 0) {
        printf("Unable to open session!\n");
        exit(-1);
    }

    // Declare subscriber
    z_owned_closure_sample_t callback;
    z_closure(&callback, data_handler, NULL, NULL);
    
    printf("Declaring Subscriber on '%s'...\n", keyexpr);
    z_owned_subscriber_t sub;
    if (z_declare_subscriber(z_loan(s), &sub, z_loan(ke), z_move(callback), NULL) < 0) {
        printf("❌ Unable to declare subscriber.\n");
        exit(-1);
    }

    printf("✅ Subscriber created for topic: %s\n", keyexpr);
    printf("📥 Listening for messages on topic '%s' for 30 seconds...\n", keyexpr);
    printf("Press Ctrl+C to stop early\n");
    printf("🔍 Subscriber is ready and waiting for messages...\n");

    // Listen for messages
    sleep(30);

    // Cleanup
    z_drop(z_move(sub));
    z_drop(z_move(s));
    
    printf("✅ Subscriber example completed\n");
    return 0;
}

const char* kind_to_str(z_sample_kind_t kind) {
    switch (kind) {
        case Z_SAMPLE_KIND_PUT:
            return "PUT";
        case Z_SAMPLE_KIND_DELETE:
            return "DELETE";
        default:
            return "UNKNOWN";
    }
}

