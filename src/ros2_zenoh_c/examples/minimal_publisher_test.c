/**
 * @file minimal_publisher_test.c
 * @brief Minimal publisher test to debug issues
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "zenoh.h"

int main() {
    printf("🧪 Minimal Publisher Test\n");
    printf("=========================\n");

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

    // Declare publisher
    printf("3. Declaring publisher...\n");
    z_owned_publisher_t pub;
    if (z_declare_publisher(z_loan(s), &pub, z_loan(ke), NULL) < 0) {
        printf("❌ Failed to declare publisher!\n");
        z_drop(z_move(s));
        return -1;
    }
    printf("✅ Publisher declared!\n");

    // Publish messages
    printf("4. Publishing messages...\n");
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
    printf("5. Cleaning up...\n");
    z_drop(z_move(pub));
    z_drop(z_move(s));
    printf("✅ Test completed!\n");
    return 0;
}
