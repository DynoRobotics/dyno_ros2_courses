/**
 * @file combined_test.c
 * @brief Combined publisher/subscriber test using same session
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include "zenoh.h"

static z_owned_session_t g_session;
static int g_messages_received = 0;

void data_handler(z_loaned_sample_t* sample, void* arg) {
    printf("🔔 CALLBACK TRIGGERED! Message #%d\n", ++g_messages_received);
    
    z_view_string_t key_string;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &key_string);

    z_owned_string_t payload_string;
    z_bytes_to_string(z_sample_payload(sample), &payload_string);

    printf("📨 Received: '%.*s' -> '%.*s'\n", 
           (int)z_string_len(z_loan(key_string)), z_string_data(z_loan(key_string)),
           (int)z_string_len(z_loan(payload_string)), z_string_data(z_loan(payload_string)));
    
    z_drop(z_move(payload_string));
}

void* publisher_thread(void* arg) {
    printf("📤 Publisher thread starting...\n");
    
    // Create key expression
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, "test/topic") < 0) {
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
    for (int i = 0; i < 5; i++) {
        char msg[64];
        sprintf(msg, "Hello from C publisher! Message %d", i);
        
        z_publisher_put_options_t options;
        z_publisher_put_options_default(&options);

        z_owned_bytes_t payload;
        z_bytes_copy_from_str(&payload, msg);
        
        printf("📤 Publishing: '%s'\n", msg);
        z_publisher_put(z_loan(pub), z_move(payload), &options);
        
        sleep(1);
    }

    // Cleanup
    z_drop(z_move(pub));
    printf("📤 Publisher thread finished!\n");
    return NULL;
}

int main() {
    printf("🧪 Combined Publisher/Subscriber Test\n");
    printf("=====================================\n");

    zc_init_log_from_env_or("info");

    // Open session
    printf("1. Opening session...\n");
    z_owned_config_t config;
    z_config_default(&config);
    
    if (z_open(&g_session, z_move(config), NULL) < 0) {
        printf("❌ Failed to open session!\n");
        return -1;
    }
    printf("✅ Session opened!\n");

    // Create key expression for subscriber
    printf("2. Creating key expression...\n");
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, "test/topic") < 0) {
        printf("❌ Invalid key expression!\n");
        z_drop(z_move(g_session));
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
    if (z_declare_subscriber(z_loan(g_session), &sub, z_loan(ke), z_move(callback), NULL) < 0) {
        printf("❌ Failed to declare subscriber!\n");
        z_drop(z_move(g_session));
        return -1;
    }
    printf("✅ Subscriber declared!\n");

    // Start publisher thread
    printf("5. Starting publisher thread...\n");
    pthread_t pub_thread;
    if (pthread_create(&pub_thread, NULL, publisher_thread, NULL) != 0) {
        printf("❌ Failed to create publisher thread!\n");
        z_drop(z_move(sub));
        z_drop(z_move(g_session));
        return -1;
    }

    // Wait for messages
    printf("6. Waiting for messages (10 seconds)...\n");
    sleep(10);

    // Wait for publisher thread to finish
    pthread_join(pub_thread, NULL);

    // Cleanup
    printf("7. Cleaning up...\n");
    z_drop(z_move(sub));
    z_drop(z_move(g_session));
    
    printf("✅ Test completed! Received %d messages.\n", g_messages_received);
    return 0;
}
