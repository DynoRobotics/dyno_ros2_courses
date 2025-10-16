#ifndef ROS2_ZENOH_PICO_H
#define ROS2_ZENOH_PICO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef struct z_session_t z_session_t;
typedef struct z_publisher_t z_publisher_t;
typedef struct z_subscriber_t z_subscriber_t;
typedef struct z_client_t z_client_t;
typedef struct z_service_t z_service_t;

// ROS2 Zenoh Pico Node
typedef struct ros2_zenoh_pico_node_t ros2_zenoh_pico_node_t;

// Error codes
typedef enum {
    ROS2_ZENOH_PICO_OK = 0,
    ROS2_ZENOH_PICO_ERROR_INVALID_ARGUMENT = -1,
    ROS2_ZENOH_PICO_ERROR_INIT_FAILED = -2,
    ROS2_ZENOH_PICO_ERROR_SESSION_FAILED = -3,
    ROS2_ZENOH_PICO_ERROR_PUBLISHER_FAILED = -4,
    ROS2_ZENOH_PICO_ERROR_SUBSCRIBER_FAILED = -5,
    ROS2_ZENOH_PICO_ERROR_CLIENT_FAILED = -6,
    ROS2_ZENOH_PICO_ERROR_SERVICE_FAILED = -7,
    ROS2_ZENOH_PICO_ERROR_MEMORY = -8,
    ROS2_ZENOH_PICO_ERROR_TIMEOUT = -9,
    ROS2_ZENOH_PICO_ERROR_NETWORK = -10
} ros2_zenoh_pico_error_t;

// Configuration structure
typedef struct {
    const char* endpoint;           // e.g., "tcp/172.18.0.2:7447"
    const char* mode;              // "client" or "peer"
    uint32_t timeout_ms;          // Timeout in milliseconds
    bool verbose;                 // Enable verbose logging
} ros2_zenoh_pico_config_t;

// Default configuration
#define ROS2_ZENOH_PICO_DEFAULT_CONFIG { \
    .endpoint = NULL, \
    .mode = "client", \
    .timeout_ms = 5000, \
    .verbose = false \
}

// Node management
ros2_zenoh_pico_node_t* ros2_zenoh_pico_node_create(const ros2_zenoh_pico_config_t* config);
int ros2_zenoh_pico_node_destroy(ros2_zenoh_pico_node_t* node);
bool ros2_zenoh_pico_node_is_valid(const ros2_zenoh_pico_node_t* node);

// Publisher
typedef struct {
    const char* topic;
    uint32_t qos_reliability;     // 0 = best effort, 1 = reliable
    uint32_t qos_durability;      // 0 = volatile, 1 = transient local
} ros2_zenoh_pico_publisher_config_t;

#define ROS2_ZENOH_PICO_DEFAULT_PUBLISHER_CONFIG { \
    .topic = NULL, \
    .qos_reliability = 0, \
    .qos_durability = 0 \
}

ros2_zenoh_pico_node_t* ros2_zenoh_pico_create_publisher(
    ros2_zenoh_pico_node_t* node,
    const ros2_zenoh_pico_publisher_config_t* config
);

int ros2_zenoh_pico_publish(
    ros2_zenoh_pico_node_t* node,
    const char* topic,
    const void* data,
    size_t data_size
);

int ros2_zenoh_pico_publish_with_attachment(
    ros2_zenoh_pico_node_t* node,
    const char* topic,
    const void* data,
    size_t data_size,
    const void* attachment,
    size_t attachment_size
);

// Subscriber
typedef void (*ros2_zenoh_pico_subscription_callback_t)(
    const char* topic,
    const void* data,
    size_t data_size,
    void* user_data
);

typedef struct {
    const char* topic;
    ros2_zenoh_pico_subscription_callback_t callback;
    void* user_data;
    uint32_t qos_reliability;
    uint32_t qos_durability;
} ros2_zenoh_pico_subscriber_config_t;

#define ROS2_ZENOH_PICO_DEFAULT_SUBSCRIBER_CONFIG { \
    .topic = NULL, \
    .callback = NULL, \
    .user_data = NULL, \
    .qos_reliability = 0, \
    .qos_durability = 0 \
}

ros2_zenoh_pico_node_t* ros2_zenoh_pico_create_subscriber(
    ros2_zenoh_pico_node_t* node,
    const ros2_zenoh_pico_subscriber_config_t* config
);

// Client (for request-response)
typedef void (*ros2_zenoh_pico_response_callback_t)(
    const char* keyexpr,
    const void* data,
    size_t data_size,
    void* user_data
);

typedef struct {
    const char* keyexpr;
    ros2_zenoh_pico_response_callback_t callback;
    void* user_data;
    uint32_t timeout_ms;
} ros2_zenoh_pico_client_config_t;

#define ROS2_ZENOH_PICO_DEFAULT_CLIENT_CONFIG { \
    .keyexpr = NULL, \
    .callback = NULL, \
    .user_data = NULL, \
    .timeout_ms = 5000 \
}

ros2_zenoh_pico_node_t* ros2_zenoh_pico_create_client(
    ros2_zenoh_pico_node_t* node,
    const ros2_zenoh_pico_client_config_t* config
);

int ros2_zenoh_pico_request(
    ros2_zenoh_pico_node_t* node,
    const char* keyexpr,
    const void* data,
    size_t data_size
);

// Service (for request-response server)
typedef void (*ros2_zenoh_pico_request_callback_t)(
    const char* keyexpr,
    const void* data,
    size_t data_size,
    void* user_data
);

typedef struct {
    const char* keyexpr;
    ros2_zenoh_pico_request_callback_t callback;
    void* user_data;
} ros2_zenoh_pico_service_config_t;

#define ROS2_ZENOH_PICO_DEFAULT_SERVICE_CONFIG { \
    .keyexpr = NULL, \
    .callback = NULL, \
    .user_data = NULL \
}

ros2_zenoh_pico_node_t* ros2_zenoh_pico_create_service(
    ros2_zenoh_pico_node_t* node,
    const ros2_zenoh_pico_service_config_t* config
);

// Utility functions
const char* ros2_zenoh_pico_get_error_string(ros2_zenoh_pico_error_t error);
int ros2_zenoh_pico_sleep_ms(uint32_t milliseconds);

// Accessor functions
void* ros2_zenoh_pico_node_get_session(ros2_zenoh_pico_node_t* node);

// Message serialization helpers (using microCDR)
int ros2_zenoh_pico_serialize_string(const char* str, void* buffer, size_t buffer_size, size_t* serialized_size);
int ros2_zenoh_pico_serialize_uint32(uint32_t value, void* buffer, size_t buffer_size, size_t* serialized_size);
int ros2_zenoh_pico_serialize_float(float value, void* buffer, size_t buffer_size, size_t* serialized_size);

int ros2_zenoh_pico_deserialize_string(const void* buffer, size_t buffer_size, char* str, size_t str_size);
int ros2_zenoh_pico_deserialize_uint32(const void* buffer, size_t buffer_size, uint32_t* value);
int ros2_zenoh_pico_deserialize_float(const void* buffer, size_t buffer_size, float* value);

#ifdef __cplusplus
}
#endif

#endif // ROS2_ZENOH_PICO_H
