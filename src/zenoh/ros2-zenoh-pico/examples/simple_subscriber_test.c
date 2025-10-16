#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <zenoh-pico.h>

void data_handler(z_loaned_sample_t *sample, void *arg) {
    (void)(arg);
    printf("DEBUG: data_handler called!\n");
    fflush(stdout);
    
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);
    z_owned_string_t value;
    z_bytes_to_string(z_sample_payload(sample), &value);
    printf(">> [Subscriber] Received ('%.*s': '%.*s')\n", (int)z_string_len(z_view_string_loan(&keystr)),
           z_string_data(z_view_string_loan(&keystr)), (int)z_string_len(z_string_loan(&value)),
           z_string_data(z_string_loan(&value)));
    fflush(stdout);
    z_string_drop(z_string_move(&value));
}

int main(int argc, char **argv) {
    const char *keyexpr = "demo/ros2_zenoh_pico/**";
    const char *mode = "client";
    char *clocator = "tcp/172.18.0.2:7447";

    z_owned_config_t config;
    z_config_default(&config);
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, mode);
    if (clocator != NULL) {
        zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, clocator);
    }

    printf("Opening session...\n");
    z_owned_session_t s;
    if (z_open(&s, z_config_move(&config), NULL) < 0) {
        printf("Unable to open session!\n");
        return -1;
    }

    // Start read and lease tasks for zenoh-pico
    if (zp_start_read_task(z_session_loan_mut(&s), NULL) < 0 || zp_start_lease_task(z_session_loan_mut(&s), NULL) < 0) {
        printf("Unable to start read and lease tasks\n");
        z_session_drop(z_session_move(&s));
        return -1;
    }

    z_owned_closure_sample_t callback;
    z_closure_sample(&callback, data_handler, NULL, NULL);

    printf("Declaring Subscriber on '%s'...\n", keyexpr);
    z_owned_subscriber_t sub;
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        printf("%s is not a valid key expression\n", keyexpr);
        return -1;
    }
    if (z_declare_subscriber(z_session_loan(&s), &sub, z_view_keyexpr_loan(&ke), z_closure_sample_move(&callback),
                             NULL) < 0) {
        printf("Unable to declare subscriber.\n");
        return -1;
    }

    printf("Press CTRL-C to quit...\n");
    while (1) {
        sleep(1);
    }

    z_subscriber_drop(z_subscriber_move(&sub));
    z_session_drop(z_session_move(&s));

    return 0;
}
