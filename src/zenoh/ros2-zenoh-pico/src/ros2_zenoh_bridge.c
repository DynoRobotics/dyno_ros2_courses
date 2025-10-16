#include "ros2_zenoh_pico/ros2_zenoh_bridge.h"
#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef MICROCDR_AVAILABLE
#include <ucdr/microcdr.h>
#endif

// Internal bridge structure
struct ros2_zenoh_bridge_t {
    ros2_zenoh_bridge_config_t config;
    ros2_zenoh_pico_node_t* zenoh_node;
    bool is_valid;
    bool is_running;
};

// Message type information for common ROS2 messages
static const ros2_message_info_t message_info_table[] = {
    {
        .message_type = "geometry_msgs/msg/Twist",
        .type_hash = "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a",
        .message_size = 24  // 6 floats * 4 bytes
    },
    {
        .message_type = "geometry_msgs/msg/Vector3", 
        .type_hash = "RIHS01_4a7b354a29a8a324c9f9ce904d36969a1eb5b805c515e434cbabac4562cb363d",
        .message_size = 12  // 3 floats * 4 bytes
    },
    {
        .message_type = "builtin_interfaces/msg/Time",
        .type_hash = "RIHS01_717e0f25a0eeb467efd904dceb35d203",
        .message_size = 8   // int32 + uint32
    },
    {
        .message_type = NULL,  // End marker
        .type_hash = NULL,
        .message_size = 0
    }
};

// Error string mapping
static const char* error_strings[] = {
    "OK",
    "Invalid argument",
    "Initialization failed",
    "Topic mapping failed", 
    "Message conversion failed",
    "Serialization failed",
    "Bridge not valid",
    "Memory allocation failed"
};

const char* ros2_zenoh_bridge_get_error_string(int error_code) {
    int index = -(int)error_code;
    if (index >= 0 && index < sizeof(error_strings) / sizeof(error_strings[0])) {
        return error_strings[index];
    }
    return "Unknown error";
}

ros2_zenoh_bridge_t* ros2_zenoh_bridge_create(const ros2_zenoh_bridge_config_t* config) {
    if (!config) {
        printf("DEBUG: Invalid config for bridge creation\n");
        return NULL;
    }

    printf("DEBUG: Creating ROS2-Zenoh bridge with endpoint: %s\n", config->zenoh_endpoint);

    ros2_zenoh_bridge_t* bridge = malloc(sizeof(ros2_zenoh_bridge_t));
    if (!bridge) {
        printf("DEBUG: Failed to allocate memory for bridge\n");
        return NULL;
    }

    // Initialize bridge
    memset(bridge, 0, sizeof(ros2_zenoh_bridge_t));
    bridge->config = *config;
    bridge->is_valid = false;
    bridge->is_running = false;

    // Create Zenoh node
    ros2_zenoh_pico_config_t zenoh_config = ROS2_ZENOH_PICO_DEFAULT_CONFIG;
    zenoh_config.endpoint = config->zenoh_endpoint;
    zenoh_config.verbose = config->verbose;

    bridge->zenoh_node = ros2_zenoh_pico_node_create(&zenoh_config);
    if (!bridge->zenoh_node) {
        printf("DEBUG: Failed to create zenoh node for bridge\n");
        free(bridge);
        return NULL;
    }

    bridge->is_valid = true;

    if (config->verbose) {
        printf("ROS2-Zenoh bridge created successfully\n");
        printf("  Zenoh endpoint: %s\n", config->zenoh_endpoint);
        printf("  ROS2 namespace: %s\n", config->ros2_namespace);
        printf("  Zenoh prefix: %s\n", config->zenoh_prefix);
    }

    return bridge;
}

int ros2_zenoh_bridge_destroy(ros2_zenoh_bridge_t* bridge) {
    if (!bridge) {
        return ROS2_ZENOH_BRIDGE_ERROR_INVALID_ARGUMENT;
    }

    if (bridge->is_running) {
        ros2_zenoh_bridge_stop(bridge);
    }

    if (bridge->zenoh_node) {
        ros2_zenoh_pico_node_destroy(bridge->zenoh_node);
    }

    free(bridge);
    return ROS2_ZENOH_BRIDGE_OK;
}

bool ros2_zenoh_bridge_is_valid(const ros2_zenoh_bridge_t* bridge) {
    return bridge && bridge->is_valid && ros2_zenoh_pico_node_is_valid(bridge->zenoh_node);
}

const char* ros2_zenoh_bridge_ros2_to_zenoh_topic(const char* ros2_topic, const char* prefix) {
    if (!ros2_topic || !prefix) {
        return NULL;
    }

    // Convert ROS2 topic to Zenoh topic
    // Example: "/turtle1/cmd_vel" -> "ros2/turtle1/cmd_vel"
    static char zenoh_topic[256];
    snprintf(zenoh_topic, sizeof(zenoh_topic), "%s%s", prefix, ros2_topic);
    return zenoh_topic;
}

const char* ros2_zenoh_bridge_zenoh_to_ros2_topic(const char* zenoh_topic, const char* prefix) {
    if (!zenoh_topic || !prefix) {
        return NULL;
    }

    // Convert Zenoh topic to ROS2 topic
    // Example: "ros2/turtle1/cmd_vel" -> "/turtle1/cmd_vel"
    static char ros2_topic[256];
    size_t prefix_len = strlen(prefix);
    
    if (strncmp(zenoh_topic, prefix, prefix_len) == 0) {
        strncpy(ros2_topic, zenoh_topic + prefix_len, sizeof(ros2_topic) - 1);
        ros2_topic[sizeof(ros2_topic) - 1] = '\0';
        return ros2_topic;
    }
    
    return NULL;
}

const ros2_message_info_t* ros2_zenoh_bridge_get_message_info(const char* message_type) {
    if (!message_type) {
        return NULL;
    }

    for (int i = 0; message_info_table[i].message_type != NULL; i++) {
        if (strcmp(message_info_table[i].message_type, message_type) == 0) {
            return &message_info_table[i];
        }
    }
    
    return NULL;
}

int ros2_zenoh_bridge_serialize_twist(const void* twist_msg, uint8_t* buffer, size_t buffer_size, size_t* serialized_size) {
#ifdef MICROCDR_AVAILABLE
    if (!twist_msg || !buffer || !serialized_size) {
        return ROS2_ZENOH_BRIDGE_ERROR_INVALID_ARGUMENT;
    }

    // Cast to our simple twist structure
    typedef struct {
        float linear_x, linear_y, linear_z;
        float angular_x, angular_y, angular_z;
    } simple_twist_t;
    
    const simple_twist_t* twist = (const simple_twist_t*)twist_msg;
    
    ucdrBuffer writer;
    ucdr_init_buffer(&writer, buffer, buffer_size);
    
    // Serialize linear velocity (Vector3)
    if (!ucdr_serialize_float(&writer, twist->linear_x) ||
        !ucdr_serialize_float(&writer, twist->linear_y) ||
        !ucdr_serialize_float(&writer, twist->linear_z)) {
        return ROS2_ZENOH_BRIDGE_ERROR_SERIALIZATION_FAILED;
    }
    
    // Serialize angular velocity (Vector3)
    if (!ucdr_serialize_float(&writer, twist->angular_x) ||
        !ucdr_serialize_float(&writer, twist->angular_y) ||
        !ucdr_serialize_float(&writer, twist->angular_z)) {
        return ROS2_ZENOH_BRIDGE_ERROR_SERIALIZATION_FAILED;
    }
    
    *serialized_size = ucdr_buffer_length(&writer);
    return ROS2_ZENOH_BRIDGE_OK;
#else
    return ROS2_ZENOH_BRIDGE_ERROR_SERIALIZATION_FAILED;
#endif
}

int ros2_zenoh_bridge_deserialize_twist(const uint8_t* buffer, size_t buffer_size, void* twist_msg) {
#ifdef MICROCDR_AVAILABLE
    if (!buffer || !twist_msg) {
        return ROS2_ZENOH_BRIDGE_ERROR_INVALID_ARGUMENT;
    }

    // Cast to our simple twist structure
    typedef struct {
        float linear_x, linear_y, linear_z;
        float angular_x, angular_y, angular_z;
    } simple_twist_t;
    
    simple_twist_t* twist = (simple_twist_t*)twist_msg;
    
    ucdrBuffer reader;
    ucdr_init_buffer(&reader, (uint8_t*)buffer, buffer_size);
    
    // Deserialize linear velocity (Vector3)
    if (!ucdr_deserialize_float(&reader, &twist->linear_x) ||
        !ucdr_deserialize_float(&reader, &twist->linear_y) ||
        !ucdr_deserialize_float(&reader, &twist->linear_z)) {
        return ROS2_ZENOH_BRIDGE_ERROR_SERIALIZATION_FAILED;
    }
    
    // Deserialize angular velocity (Vector3)
    if (!ucdr_deserialize_float(&reader, &twist->angular_x) ||
        !ucdr_deserialize_float(&reader, &twist->angular_y) ||
        !ucdr_deserialize_float(&reader, &twist->angular_z)) {
        return ROS2_ZENOH_BRIDGE_ERROR_SERIALIZATION_FAILED;
    }
    
    return ROS2_ZENOH_BRIDGE_OK;
#else
    return ROS2_ZENOH_BRIDGE_ERROR_SERIALIZATION_FAILED;
#endif
}

int ros2_zenoh_bridge_add_topic_mapping(
    ros2_zenoh_bridge_t* bridge,
    const char* ros2_topic,
    const char* message_type,
    ros2_zenoh_bridge_mode_t mode
) {
    if (!bridge || !ros2_topic || !message_type) {
        return ROS2_ZENOH_BRIDGE_ERROR_INVALID_ARGUMENT;
    }

    if (!ros2_zenoh_bridge_is_valid(bridge)) {
        return ROS2_ZENOH_BRIDGE_ERROR_BRIDGE_NOT_VALID;
    }

    const ros2_message_info_t* msg_info = ros2_zenoh_bridge_get_message_info(message_type);
    if (!msg_info) {
        printf("Warning: Unknown message type: %s\n", message_type);
        return ROS2_ZENOH_BRIDGE_ERROR_MESSAGE_CONVERSION_FAILED;
    }

    const char* zenoh_topic = ros2_zenoh_bridge_ros2_to_zenoh_topic(ros2_topic, bridge->config.zenoh_prefix);

    if (bridge->config.verbose) {
        printf("Added topic mapping:\n");
        printf("  ROS2 topic: %s\n", ros2_topic);
        printf("  Zenoh topic: %s\n", zenoh_topic);
        printf("  Message type: %s\n", message_type);
        printf("  Mode: %d\n", mode);
    }

    return ROS2_ZENOH_BRIDGE_OK;
}

int ros2_zenoh_bridge_start(ros2_zenoh_bridge_t* bridge) {
    if (!bridge) {
        return ROS2_ZENOH_BRIDGE_ERROR_INVALID_ARGUMENT;
    }

    if (!ros2_zenoh_bridge_is_valid(bridge)) {
        return ROS2_ZENOH_BRIDGE_ERROR_BRIDGE_NOT_VALID;
    }

    bridge->is_running = true;

    if (bridge->config.verbose) {
        printf("ROS2-Zenoh bridge started\n");
    }

    return ROS2_ZENOH_BRIDGE_OK;
}

int ros2_zenoh_bridge_stop(ros2_zenoh_bridge_t* bridge) {
    if (!bridge) {
        return ROS2_ZENOH_BRIDGE_ERROR_INVALID_ARGUMENT;
    }

    bridge->is_running = false;

    if (bridge->config.verbose) {
        printf("ROS2-Zenoh bridge stopped\n");
    }

    return ROS2_ZENOH_BRIDGE_OK;
}

void* ros2_zenoh_bridge_get_zenoh_node(ros2_zenoh_bridge_t* bridge) {
    if (!bridge) {
        return NULL;
    }
    return bridge->zenoh_node;
}

int ros2_zenoh_bridge_sleep_ms(uint32_t milliseconds) {
    usleep(milliseconds * 1000);
    return ROS2_ZENOH_BRIDGE_OK;
}
