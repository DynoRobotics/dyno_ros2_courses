/**
 * @file publisher_example_zenoh.c
 * @brief Example publisher using direct Zenoh C API
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include "zenoh.h"

#define DEFAULT_KEYEXPR "turtle1/cmd_vel"
#define DEFAULT_VALUE "Twist message from C!"

int main(int argc, char** argv) {
    zc_init_log_from_env_or("error");

    printf("🚀 Starting ROS 2 Zenoh C Publisher Example\n");

    // Parse arguments
    char* keyexpr = DEFAULT_KEYEXPR;
    char* value = DEFAULT_VALUE;
    
    if (argc > 1) keyexpr = argv[1];
    if (argc > 2) value = argv[2];

    // Open session
    printf("Opening session...\n");
    z_owned_config_t config;
    z_config_default(&config);
    
    z_owned_session_t s;
    if (z_open(&s, z_move(config), NULL) < 0) {
        printf("Unable to open session!\n");
        exit(-1);
    }

    // Declare publisher
    printf("Declaring Publisher on '%s'...\n", keyexpr);
    z_owned_publisher_t pub;
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str(&ke, keyexpr);
    
    if (z_declare_publisher(z_loan(s), &pub, z_loan(ke), NULL) < 0) {
        printf("Unable to declare Publisher for key expression!\n");
        exit(-1);
    }

    printf("✅ Publisher created for topic: %s\n", keyexpr);
    printf("📤 Publishing Twist messages for 10 seconds...\n");

    // Publish messages
    char buf[256] = {0};
    time_t start_time = time(NULL);
    
    while (time(NULL) - start_time < 10) {
        sprintf(buf, "[%ld] %s", time(NULL), value);
        printf("Putting Data ('%s': '%s')...\n", keyexpr, buf);
        
        z_publisher_put_options_t options;
        z_publisher_put_options_default(&options);

        z_owned_bytes_t payload;
        z_bytes_copy_from_str(&payload, buf);
        
        // Optional encoding
        z_owned_encoding_t encoding;
        z_encoding_clone(&encoding, z_encoding_text_plain());
        options.encoding = z_move(encoding);

        z_publisher_put(z_loan(pub), z_move(payload), &options);
        
        sleep(1);
    }

    // Send stop command
    printf("🛑 Sending stop command...\n");
    z_publisher_put_options_t options;
    z_publisher_put_options_default(&options);
    
    z_owned_bytes_t payload;
    z_bytes_copy_from_str(&payload, "STOP");
    
    z_owned_encoding_t encoding;
    z_encoding_clone(&encoding, z_encoding_text_plain());
    options.encoding = z_move(encoding);

    z_publisher_put(z_loan(pub), z_move(payload), &options);

    // Cleanup
    z_drop(z_move(pub));
    z_drop(z_move(s));
    
    printf("✅ Publisher example completed\n");
    return 0;
}

