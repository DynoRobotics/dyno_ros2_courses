#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include "zenoh-pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Internal subscriber data structure
typedef struct {
    ros2_zenoh_pico_subscription_callback_t callback;
    void* user_data;
} subscriber_data_t;

// Subscriber entry structure (matches node.c)
typedef struct subscriber_entry_t {
    z_owned_subscriber_t subscriber;
    void* callback_data;
    struct subscriber_entry_t* next;
} subscriber_entry_t;

// Internal node structure (matches node.c)
struct ros2_zenoh_pico_node_t {
    z_owned_session_t session;
    bool is_valid;
    ros2_zenoh_pico_config_t config;
    subscriber_entry_t* subscribers;  // Linked list of subscribers
};

// Callback wrapper that bridges to user callback
void subscriber_callback_wrapper(z_loaned_sample_t *sample, void *arg) {
    printf("DEBUG: subscriber_callback_wrapper called!\n");
    fflush(stdout);
    
    subscriber_data_t* sub_data = (subscriber_data_t*)arg;
    
    if (!sub_data || !sub_data->callback) {
        printf("DEBUG: sub_data or callback is NULL!\n");
        fflush(stdout);
        return;
    }

    // Extract topic from keyexpr
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);
    const char* topic = z_string_data(z_view_string_loan(&keystr));
    size_t topic_len = z_string_len(z_view_string_loan(&keystr));
    
    // Extract payload as string
    z_owned_string_t value;
    z_bytes_to_string(z_sample_payload(sample), &value);
    const char* data = z_string_data(z_string_loan(&value));
    size_t data_size = z_string_len(z_string_loan(&value));
    
    printf("DEBUG: Calling user callback with topic: %.*s, data: %.*s\n", (int)topic_len, topic, (int)data_size, data);
    fflush(stdout);
    
    // Call user callback
    sub_data->callback(topic, data, data_size, sub_data->user_data);
    
    // Clean up
    z_string_drop(z_string_move(&value));
}

ros2_zenoh_pico_node_t* ros2_zenoh_pico_create_subscriber(
    ros2_zenoh_pico_node_t* node,
    const ros2_zenoh_pico_subscriber_config_t* config
) {
    if (!node || !config || !config->topic || !config->callback) {
        printf("DEBUG: Invalid parameters for subscriber creation\n");
        return NULL;
    }

    if (!ros2_zenoh_pico_node_is_valid(node)) {
        printf("DEBUG: Node is not valid for subscriber creation\n");
        return NULL;
    }

    printf("DEBUG: Creating subscriber for topic: %s\n", config->topic);

    // Allocate subscriber data
    subscriber_data_t* sub_data = malloc(sizeof(subscriber_data_t));
    if (!sub_data) {
        return NULL;
    }

    sub_data->callback = config->callback;
    sub_data->user_data = config->user_data;

    // Create keyexpr view
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, config->topic) < 0) {
        if (node->config.verbose) {
            printf("Invalid key expression: %s\n", config->topic);
        }
        free(sub_data);
        return NULL;
    }

    // Create subscriber entry first
    subscriber_entry_t* entry = malloc(sizeof(subscriber_entry_t));
    if (!entry) {
        free(sub_data);
        return NULL;
    }
    
    // Create closure for callback (using wrapper that calls user callback)
    printf("DEBUG: Creating closure with callback wrapper function\n");
    fflush(stdout);
    z_owned_closure_sample_t callback;
    z_closure_sample(&callback, subscriber_callback_wrapper, NULL, sub_data);
    printf("DEBUG: Closure created successfully\n");
    fflush(stdout);

    // Create subscriber directly in the entry
    printf("DEBUG: Declaring subscriber with zenoh\n");
    fflush(stdout);
    int result = z_declare_subscriber(z_session_loan(&node->session), &entry->subscriber, z_view_keyexpr_loan(&ke), z_closure_sample_move(&callback), NULL);
    printf("DEBUG: z_declare_subscriber returned: %d\n", result);
    fflush(stdout);
    if (result < 0) {
        if (node->config.verbose) {
            printf("Unable to declare subscriber for topic: %s\n", config->topic);
        }
        free(entry);
        free(sub_data);
        return NULL;
    }

    // Add to node's subscriber list
    entry->callback_data = sub_data;
    entry->next = node->subscribers;
    node->subscribers = entry;

    if (node->config.verbose) {
        printf("Subscriber created for topic: %s\n", config->topic);
    }
    
    return node;
}