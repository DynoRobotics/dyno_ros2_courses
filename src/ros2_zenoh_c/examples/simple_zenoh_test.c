/**
 * @file simple_zenoh_test.c
 * @brief Simple test to verify Zenoh C library connectivity
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "zenoh.h"

int main() {
    printf("🧪 Simple Zenoh C Library Test\n");
    printf("=============================\n");

    // Initialize logging
    zc_init_log_from_env_or("info");

    // Open session
    printf("Opening session...\n");
    z_owned_config_t config;
    z_config_default(&config);
    
    z_owned_session_t s;
    if (z_open(&s, z_move(config), NULL) < 0) {
        printf("❌ Unable to open session!\n");
        return -1;
    }
    printf("✅ Session opened successfully!\n");

    // Test key expression
    printf("Testing key expression...\n");
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, "test/topic") < 0) {
        printf("❌ Invalid key expression!\n");
        z_drop(z_move(s));
        return -1;
    }
    printf("✅ Key expression 'test/topic' is valid!\n");

    // Cleanup
    z_drop(z_move(s));
    printf("✅ Test completed successfully!\n");
    return 0;
}
