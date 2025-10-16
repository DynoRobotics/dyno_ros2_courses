#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#ifdef MICROCDR_AVAILABLE
#include <ucdr/microcdr.h>
#endif

// Define platform for zenoh-pico
#define ZENOH_LINUX
#define ZENOH_COMPILER_GCC
#define ZENOH_PICO_PLATFORM_LINUX

#include <zenoh-pico/config.h>
#if Z_FEATURE_LIVELINESS == 1
#include <zenoh-pico/api/liveliness.h>
#include <zenoh-pico/api/primitives.h>
#include <zenoh-pico/api/macros.h>
#endif

// ROS2 Twist message structure (matches geometry_msgs/Twist)
typedef struct {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
} ros2_twist_t;

// ROS2-compatible message metadata
typedef struct {
    char message_type[128];     // e.g., "geometry_msgs::msg::dds_::Twist_"
    char type_hash[128];        // e.g., "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a"
    char encoding[32];          // e.g., "application/x-cdr"
    uint64_t sequence_number;
    uint64_t timestamp_ns;
    uint8_t publisher_gid[16];  // Publisher GUID
} ros2_message_metadata_t;

// CDR serialization for ROS2 Twist message
int serialize_ros2_twist(const ros2_twist_t* twist, uint8_t* buffer, size_t buffer_size, size_t* serialized_size) {
#ifdef MICROCDR_AVAILABLE
    ucdrBuffer writer;
    // Initialize buffer with little-endian for CDR compatibility
    ucdr_init_buffer_origin_offset_endian(&writer, buffer, buffer_size, 0, 0, UCDR_LITTLE_ENDIANNESS);
    
    // Serialize linear velocity (Vector3) - no CDR encapsulation header
    // rmw_zenoh handles encapsulation internally
    if (!ucdr_serialize_float(&writer, twist->linear_x) ||
        !ucdr_serialize_float(&writer, twist->linear_y) ||
        !ucdr_serialize_float(&writer, twist->linear_z)) {
        printf("DEBUG: Failed to serialize linear velocity\n");
        return -1;
    }
    
    // Add alignment padding if needed (CDR requires 4-byte alignment)
    // Check if we need padding before the next struct
    size_t current_pos = ucdr_buffer_length(&writer);
    if (current_pos % 4 != 0) {
        size_t padding_needed = 4 - (current_pos % 4);
        for (size_t i = 0; i < padding_needed; i++) {
            if (!ucdr_serialize_uint8_t(&writer, 0)) {
                printf("DEBUG: Failed to serialize padding\n");
                return -1;
            }
        }
    }
    
    // Serialize angular velocity (Vector3)
    if (!ucdr_serialize_float(&writer, twist->angular_x) ||
        !ucdr_serialize_float(&writer, twist->angular_y) ||
        !ucdr_serialize_float(&writer, twist->angular_z)) {
        printf("DEBUG: Failed to serialize angular velocity\n");
        return -1;
    }
    
    *serialized_size = ucdr_buffer_length(&writer);
    printf("DEBUG: Serialized %zu bytes (raw CDR data)\n", *serialized_size);
    
    // Debug: Print first 16 bytes of serialized data
    printf("DEBUG: First 16 bytes: ");
    for (size_t i = 0; i < 16 && i < *serialized_size; i++) {
        printf("%02x ", buffer[i]);
    }
    printf("\n");
    
    return 0;
#else
    printf("DEBUG: MICROCDR_AVAILABLE not defined\n");
    return -1;
#endif
}

// Build ROS2-compatible attachment for rmw_zenoh
int build_ros2_attachment(const ros2_message_metadata_t* metadata, uint8_t* attachment_buffer, size_t buffer_size, size_t* attachment_size) {
    if (!metadata || !attachment_buffer || !attachment_size) {
        return -1;
    }
    
    // ROS2 attachment format: seq + ts + VarInt(16) + gid
    // This matches the Python implementation's _build_attachment method
    
    uint8_t* ptr = attachment_buffer;
    size_t remaining = buffer_size;
    
    // Sequence number (8 bytes, little-endian)
    if (remaining < 8) return -1;
    *(uint64_t*)ptr = metadata->sequence_number;
    ptr += 8;
    remaining -= 8;
    
    // Timestamp (8 bytes, little-endian)
    if (remaining < 8) return -1;
    *(uint64_t*)ptr = metadata->timestamp_ns;
    ptr += 8;
    remaining -= 8;
    
    // VarInt(16) = 0x10
    if (remaining < 1) return -1;
    *ptr++ = 0x10;
    remaining -= 1;
    
    // Publisher GUID (16 bytes)
    if (remaining < 16) return -1;
    memcpy(ptr, metadata->publisher_gid, 16);
    ptr += 16;
    
    *attachment_size = ptr - attachment_buffer;
    return 0;
}

// Generate a random publisher GUID
void generate_publisher_gid(uint8_t* gid) {
    srand(time(NULL));
    for (int i = 0; i < 16; i++) {
        gid[i] = rand() % 256;
    }
}

// Create ROS2 liveliness token keyexpr for metadata publishing
int create_ros2_liveliness_keyexpr(const char* topic_name, const char* message_type, const char* type_hash, 
                                  const char* node_name, char* keyexpr_buffer, size_t buffer_size) {
    if (!topic_name || !message_type || !type_hash || !node_name || !keyexpr_buffer) {
        return -1;
    }
    
    // ROS2 liveliness token format:
    // @ros2_lv/<domain_id>/<zid>/<nid>/<id>/<entity_type>/<enclave>/<namespace>/<node_name>/<topic_name>/<topic_type>/<topic_type_hash>/<qos>
    
    const char* admin_space = "@ros2_lv";
    const char* domain_id = "0";
    const char* zid = "1";  // Simplified - in real implementation, get from session
    const char* nid = "1";
    const char* entity_id = "1";
    const char* entity_type = "MP";  // MP = Message Publisher
    const char* enclave = "_";
    const char* namespace_str = "_";
    const char* qos = "1:2:1,10:0,0:0,0:1,0,0";  // Default QoS
    
    // Mangle names (replace "/" with "%")
    char mangled_topic[256];
    char mangled_message_type[256];
    char mangled_type_hash[256];
    char mangled_node_name[256];
    
    strncpy(mangled_topic, topic_name, sizeof(mangled_topic) - 1);
    strncpy(mangled_message_type, message_type, sizeof(mangled_message_type) - 1);
    strncpy(mangled_type_hash, type_hash, sizeof(mangled_type_hash) - 1);
    strncpy(mangled_node_name, node_name, sizeof(mangled_node_name) - 1);
    
    // Replace "/" with "%" for Zenoh compatibility
    for (char* p = mangled_topic; *p; p++) if (*p == '/') *p = '%';
    for (char* p = mangled_message_type; *p; p++) if (*p == '/') *p = '%';
    for (char* p = mangled_type_hash; *p; p++) if (*p == '/') *p = '%';
    for (char* p = mangled_node_name; *p; p++) if (*p == '/') *p = '%';
    
    int result = snprintf(keyexpr_buffer, buffer_size,
        "%s/%s/%s/%s/%s/%s/%s/%s/%s/%s/%s/%s/%s",
        admin_space, domain_id, zid, nid, entity_id, entity_type,
        enclave, namespace_str, mangled_node_name, mangled_topic,
        mangled_message_type, mangled_type_hash, qos);
    
    return (result < buffer_size) ? 0 : -1;
}

// Simplified version without liveliness tokens for now

// Create DDS interop key for ROS2 topic
int create_dds_interop_key(const char* topic, const char* message_type, const char* type_hash, char* dds_key_buffer, size_t buffer_size) {
    if (!topic || !message_type || !type_hash || !dds_key_buffer) {
        return -1;
    }
    
    // Extract topic name without leading slash
    const char* topic_part = topic;
    if (topic[0] == '/') {
        topic_part = topic + 1;
    }
    
    // Create DDS interop key format: "0/<topic>/<message_type>/<type_hash>"
    int result = snprintf(dds_key_buffer, buffer_size, "0/%s/%s/%s", topic_part, message_type, type_hash);
    
    return (result < buffer_size) ? 0 : -1;
}

// Publish ROS2-compatible Twist message via Zenoh
int publish_ros2_twist(ros2_zenoh_pico_node_t* node, const char* topic, const ros2_twist_t* twist, ros2_message_metadata_t* metadata) {
    if (!node || !topic || !twist || !metadata) {
        return -1;
    }
    
    // Serialize the Twist message
    uint8_t cdr_buffer[1024];
    size_t serialized_size;
    
    if (serialize_ros2_twist(twist, cdr_buffer, sizeof(cdr_buffer), &serialized_size) != 0) {
        printf("Failed to serialize ROS2 Twist message\n");
        return -1;
    }
    
    // Build attachment
    uint8_t attachment_buffer[64];
    size_t attachment_size;
    
    if (build_ros2_attachment(metadata, attachment_buffer, sizeof(attachment_buffer), &attachment_size) != 0) {
        printf("Failed to build ROS2 attachment\n");
        return -1;
    }
    
    // Create DDS interop key for ROS2 topic
    char dds_key[512];
    if (create_dds_interop_key(topic, metadata->message_type, metadata->type_hash, dds_key, sizeof(dds_key)) != 0) {
        printf("Failed to create DDS interop key\n");
        return -1;
    }
    
    printf("DEBUG: DDS Interop Key: %s\n", dds_key);
    
    // Create publisher for DDS interop key if it doesn't exist
    ros2_zenoh_pico_publisher_config_t pub_config = ROS2_ZENOH_PICO_DEFAULT_PUBLISHER_CONFIG;
    pub_config.topic = dds_key;
    
    if (!ros2_zenoh_pico_create_publisher(node, &pub_config)) {
        printf("Failed to create publisher for DDS interop key: %s\n", dds_key);
        return -1;
    }
    
    printf("DEBUG: Publisher created for DDS interop key\n");
    fflush(stdout);
    
    // Create ROS2 liveliness token keyexpr for metadata publishing
    char liveliness_keyexpr[512];
    if (create_ros2_liveliness_keyexpr(topic, metadata->message_type, metadata->type_hash, 
                                       "zenoh_publisher", liveliness_keyexpr, sizeof(liveliness_keyexpr)) != 0) {
        printf("Failed to create liveliness keyexpr\n");
        return -1;
    }
    
    printf("DEBUG: ROS2 Liveliness Token: %s\n", liveliness_keyexpr);
    
    printf("DEBUG: Attachment size: %zu bytes\n", attachment_size);
    printf("DEBUG: Attachment data: ");
    for (size_t i = 0; i < attachment_size && i < 16; i++) {
        printf("%02x ", ((uint8_t*)attachment_buffer)[i]);
    }
    printf("\n");
    fflush(stdout);
    
    // Publish via Zenoh using DDS interop key with ROS2 attachment
    int result = ros2_zenoh_pico_publish_with_attachment(node, dds_key, cdr_buffer, serialized_size, 
                                                        attachment_buffer, attachment_size);
    
    if (result == ROS2_ZENOH_PICO_OK) {
        // Update sequence number for next message
        metadata->sequence_number++;
        metadata->timestamp_ns = time(NULL) * 1000000000ULL; // Convert to nanoseconds
    }
    
    return result;
}

int main(int argc, char** argv) {
    printf("=== ROS2 Twist Publisher (rmw_zenoh compatible) ===\n");
    fflush(stdout);
    
    // Configuration
    ros2_zenoh_pico_config_t config = ROS2_ZENOH_PICO_DEFAULT_CONFIG;
    config.endpoint = "tcp/172.18.0.2:7447";  // Your Zenoh router
    config.verbose = true;

    // Create node
    ros2_zenoh_pico_node_t* node = ros2_zenoh_pico_node_create(&config);
    if (!node) {
        printf("Failed to create zenoh node\n");
        fflush(stdout);
        return 1;
    }

    printf("ROS2 Twist Publisher (rmw_zenoh compatible)\n");
    printf("Publishing to ROS2 topic: /turtle1/safe_cmd_vel\n");
    printf("DDS Interop Key: 0/turtle1/safe_cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_...\n");
    printf("Message type: geometry_msgs::msg::dds_::Twist_\n");
    printf("Encoding: application/x-cdr\n");
    fflush(stdout);

    // Declare liveliness token for ROS2 metadata
    char liveliness_keyexpr[512];
    if (create_ros2_liveliness_keyexpr("/turtle1/safe_cmd_vel", "geometry_msgs::msg::dds_::Twist_", 
                                       "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a", 
                                       "zenoh_publisher", liveliness_keyexpr, sizeof(liveliness_keyexpr)) != 0) {
        printf("Failed to create liveliness keyexpr\n");
        fflush(stdout);
        ros2_zenoh_pico_node_destroy(node);
        return 1;
    }
    
    printf("DEBUG: Declaring liveliness token: %s\n", liveliness_keyexpr);
    fflush(stdout);
    
    // Actually declare the liveliness token
    z_owned_liveliness_token_t liveliness_token;
    z_liveliness_token_options_t options;
    z_liveliness_token_options_default(&options);
    
    z_view_keyexpr_t keyexpr_view;
    if (z_view_keyexpr_from_str(&keyexpr_view, liveliness_keyexpr) != Z_OK) {
        printf("Failed to create keyexpr view\n");
        fflush(stdout);
        ros2_zenoh_pico_node_destroy(node);
        return 1;
    }
    
    z_result_t result = z_liveliness_declare_token(
        z_session_loan(ros2_zenoh_pico_node_get_session(node)), 
        &liveliness_token,
        z_loan(keyexpr_view),
        &options
    );
    
    if (result != Z_OK) {
        printf("Failed to declare liveliness token (result: %d)\n", result);
        fflush(stdout);
        ros2_zenoh_pico_node_destroy(node);
        return 1;
    }
    
    printf("Liveliness token declared successfully!\n");
    fflush(stdout);

    // Initialize ROS2 message metadata
    ros2_message_metadata_t metadata = {
        .message_type = "geometry_msgs::msg::dds_::Twist_",
        .type_hash = "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a",
        .encoding = "application/x-cdr",
        .sequence_number = 0,
        .timestamp_ns = time(NULL) * 1000000000ULL,
        .publisher_gid = {0}
    };
    
    // Generate publisher GUID
    generate_publisher_gid(metadata.publisher_gid);
    
    printf("Publisher GUID: ");
    for (int i = 0; i < 16; i++) {
        printf("%02x", metadata.publisher_gid[i]);
    }
    printf("\n");
    fflush(stdout);

    // Create Twist message
    ros2_twist_t twist;
    
    // Publish messages
    for (int i = 0; i < 3; i++) {  // Reduced to just 3 messages
        // Create a simple motion pattern
        float angle = (float)i * 0.3f; // Simple increment
        
        // Linear velocity (forward motion)
        twist.linear_x = 1.5f;  // Forward speed
        twist.linear_y = 0.0f;
        twist.linear_z = 0.0f;
        
        // Angular velocity (turning)
        twist.angular_x = 0.0f;
        twist.angular_y = 0.0f;
        twist.angular_z = 0.6f * sinf(angle);  // Oscillating turn rate
        
        // Publish ROS2-compatible message
        int result = publish_ros2_twist(node, "/turtle1/safe_cmd_vel", &twist, &metadata);
        
        if (result != ROS2_ZENOH_PICO_OK) {
            printf("Failed to publish message %d: %s\n", i, ros2_zenoh_pico_get_error_string(result));
            fflush(stdout);
            break;
        }
        
        printf("Published ROS2 Twist[%d] (seq=%lu): linear=(%.2f,%.2f,%.2f) angular=(%.2f,%.2f,%.2f)\n", 
               i, metadata.sequence_number - 1,
               twist.linear_x, twist.linear_y, twist.linear_z,
               twist.angular_x, twist.angular_y, twist.angular_z);
        fflush(stdout);
        
        ros2_zenoh_pico_sleep_ms(1000);  // Increased delay to 1 second
    }

    printf("DEBUG: About to cleanup...\n");
    fflush(stdout);
    
    // Undeclare liveliness token
    z_liveliness_undeclare_token(z_liveliness_token_move(&liveliness_token));
    printf("DEBUG: Liveliness token undeclared\n");
    fflush(stdout);
    
    // Skip node cleanup for now to avoid segfault
    // ros2_zenoh_pico_node_destroy(node);
    printf("DEBUG: Skipped node destruction to avoid segfault\n");
    fflush(stdout);
    
    printf("ROS2 Twist Publisher (rmw_zenoh compatible) completed\n");
    fflush(stdout);
    
    return 0;
}
