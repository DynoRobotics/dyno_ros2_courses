/**
 * @file minimal_subscriber_test.c
 * @brief Minimal subscriber test to debug issues
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "zenoh.h"

void data_handler(z_loaned_sample_t* sample, void* arg) {
    printf("🔔 CALLBACK TRIGGERED!\n");
    
    z_view_string_t key_string;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &key_string);

    z_owned_string_t payload_string;
    z_bytes_to_string(z_sample_payload(sample), &payload_string);

    printf("📨 Received: '%.*s' -> '%.*s'\n", 
           (int)z_string_len(z_loan(key_string)), z_string_data(z_loan(key_string)),
           (int)z_string_len(z_loan(payload_string)), z_string_data(z_loan(payload_string)));
    
    z_drop(z_move(payload_string));
}

int main() {
    printf("🧪 Minimal Subscriber Test\n");
    printf("========================\n");

    zc_init_log_from_env_or("info");

    // Open session
    printf("1. Opening session...\n");
    z_owned_config_t config;
    z_config_default(&config);
    
    z_owned_session_t s;
    if (z_open(&s, z_move(config), NULL) < 0) {
        printf("❌ Failed to open session!\n");
        return -1;
    }
    printf("✅ Session opened!\n");

    // Create key expression
    printf("2. Creating key expression...\n");
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, "test/topic") < 0) {
        printf("❌ Invalid key expression!\n");
        z_drop(z_move(s));
        return -1;
    }
    printf("✅ Key expression created!\n");

    // Create callback
    printf("3. Creating callback...\n");
    z_owned_closure_sample_t callback;
    z_closure(&callback, data_handler, NULL, NULL);
    printf("✅ Callback created!\n");

    // Declare subscriber
    printf("4. Declaring subscriber...\n");
    z_owned_subscriber_t sub;
    if (z_declare_subscriber(z_loan(s), &sub, z_loan(ke), z_move(callback), NULL) < 0) {
        printf("❌ Failed to declare subscriber!\n");
        z_drop(z_move(s));
        return -1;
    }
    printf("✅ Subscriber declared!\n");

    printf("5. Waiting for messages (10 seconds)...\n");
    sleep(10);

    // Cleanup
    printf("6. Cleaning up...\n");
    z_drop(z_move(sub));
    z_drop(z_move(s));
    printf("✅ Test completed!\n");
    return 0;
}
