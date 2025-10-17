/**
 * @file ros2_zenoh_rmw_subscriber.c
 * @brief ROS 2 Zenoh RMW Subscriber Example
 * 
 * This example demonstrates how to receive and deserialize ROS 2 Twist messages
 * using Zenoh with proper CDR encapsulation format.
 */

#include "ros2_zenoh_pico/ros2_zenoh_pico.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#ifdef MICROCDR_AVAILABLE
#include <ucdr/microcdr.h>
#endif

// Define platform for zenoh-pico
#define ZENOH_LINUX
#define ZENOH_COMPILER_GCC
#define ZENOH_PICO_PLATFORM_LINUX

#include <zenoh-pico/config.h>
#if Z_FEATURE_SUBSCRIPTION == 1
#include <zenoh-pico/api/liveliness.h>
#include <zenoh-pico/api/primitives.h>
#include <zenoh-pico/api/macros.h>
#endif

// ROS2 Vector3 message structure (matches geometry_msgs/Vector3)
typedef struct {
    double x;  // float64 in ROS 2
    double y;  // float64 in ROS 2
    double z;  // float64 in ROS 2
} ros2_vector3_t;

// ROS2 Twist message structure (matches geometry_msgs/Twist)
typedef struct {
    ros2_vector3_t linear;
    ros2_vector3_t angular;
} ros2_twist_t;

// Create ROS2 liveliness token keyexpr for subscriber metadata
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
    const char* entity_type = "MS";  // MS = Message Subscriber (vs MP = Message Publisher)
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

// CDR deserialization for ROS2 Twist message
int deserialize_ros2_twist(const uint8_t* buffer, size_t buffer_size, ros2_twist_t* twist) {
#ifdef MICROCDR_AVAILABLE
    // Check the header
    if (buffer_size < 4) {
        printf("DEBUG: Buffer too small for CDR header\n");
        return -1;
    }
    
    printf("DEBUG: CDR Header - Bytes: %02x %02x %02x %02x\n", 
           buffer[0], buffer[1], buffer[2], buffer[3]);
    
    ucdrBuffer reader;
    // Initialize buffer starting after the 4-byte CDR header
    ucdr_init_buffer_origin_offset_endian(&reader, (uint8_t*)buffer + 4, buffer_size - 4, 0, 0, UCDR_LITTLE_ENDIANNESS);
    
    // Deserialize linear velocity (Vector3)
    if (!ucdr_deserialize_double(&reader, &twist->linear.x) ||
        !ucdr_deserialize_double(&reader, &twist->linear.y) ||
        !ucdr_deserialize_double(&reader, &twist->linear.z)) {
        printf("DEBUG: Failed to deserialize linear velocity\n");
        return -1;
    }
    
    // Deserialize angular velocity (Vector3)
    if (!ucdr_deserialize_double(&reader, &twist->angular.x) ||
        !ucdr_deserialize_double(&reader, &twist->angular.y) ||
        !ucdr_deserialize_double(&reader, &twist->angular.z)) {
        printf("DEBUG: Failed to deserialize angular velocity\n");
        return -1;
    }
    
    printf("DEBUG: Deserialized Twist message successfully\n");
    return 0;
#else
    printf("DEBUG: MICROCDR_AVAILABLE not defined\n");
    return -1;
#endif
}

// Callback function for received messages
void twist_callback(z_loaned_sample_t* sample, void* arg) {
    printf("🔔 Received message!\n");
    
    // Get key expression
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);
    printf("📝 Key: %.*s\n", (int)z_string_len(z_view_string_loan(&keystr)), 
           z_string_data(z_view_string_loan(&keystr)));
    
    // Get payload
    const z_loaned_bytes_t* payload = z_sample_payload(sample);
    if (payload == NULL) {
        printf("❌ No payload in message\n");
        return;
    }
    
    // Convert bytes to slice to access raw data
    z_owned_slice_t slice;
    if (z_bytes_to_slice(payload, &slice) < 0) {
        printf("❌ Failed to convert payload to slice\n");
        return;
    }
    
    const z_loaned_slice_t* loaned_slice = z_slice_loan(&slice);
    printf("📦 Payload size: %zu bytes\n", z_slice_len(loaned_slice));
    
    // Debug: Print first 16 bytes of payload
    printf("DEBUG: First 16 bytes: ");
    for (size_t i = 0; i < 16 && i < z_slice_len(loaned_slice); i++) {
        printf("%02x ", z_slice_data(loaned_slice)[i]);
    }
    printf("\n");
    
    // Deserialize the Twist message
    ros2_twist_t twist;
    int result = deserialize_ros2_twist(z_slice_data(loaned_slice), z_slice_len(loaned_slice), &twist);
    
    // Clean up slice
    z_slice_drop(z_slice_move(&slice));
    
    if (result == 0) {
        printf("✅ Successfully deserialized Twist message:\n");
        printf("   Linear:  x=%.6f, y=%.6f, z=%.6f\n", 
               twist.linear.x, twist.linear.y, twist.linear.z);
        printf("   Angular: x=%.6f, y=%.6f, z=%.6f\n", 
               twist.angular.x, twist.angular.y, twist.angular.z);
        
        // Calculate speed and turn rate
        double linear_speed = sqrt(twist.linear.x * twist.linear.x + 
                                  twist.linear.y * twist.linear.y + 
                                  twist.linear.z * twist.linear.z);
        double angular_speed = sqrt(twist.angular.x * twist.angular.x + 
                                   twist.angular.y * twist.angular.y + 
                                   twist.angular.z * twist.angular.z);
        
        printf("   Speed: %.3f m/s, Turn rate: %.3f rad/s\n", linear_speed, angular_speed);
    } else {
        printf("❌ Failed to deserialize Twist message\n");
    }
    
    printf("---\n");
}

int main(int argc, char** argv) {
    printf("🚀 Starting ROS 2 Zenoh RMW Subscriber Example\n");
    printf("==============================================\n");
    
    // Default configuration
    char* topic = "turtle1/cmd_vel";  // Use valid Zenoh key expression
    int duration = 30;  // seconds
    
    // Parse command line arguments
    if (argc > 1) {
        topic = argv[1];
    }
    if (argc > 2) {
        duration = atoi(argv[2]);
    }
    
    printf("📥 Subscribing to topic: %s\n", topic);
    printf("⏱️  Listening for %d seconds\n", duration);
    
#if Z_FEATURE_SUBSCRIPTION == 1
    // Initialize Zenoh
    z_owned_config_t config;
    z_config_default(&config);
    
    // Configure endpoint to match publisher
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, "tcp/0.0.0.0:7447");
    
    // Open Zenoh session
    z_owned_session_t s;
    if (z_open(&s, z_config_move(&config), NULL) < 0) {
        printf("❌ Failed to open Zenoh session\n");
        return -1;
    }
    
    printf("✅ Zenoh session opened\n");
    
    // Declare liveliness token for ROS2 subscriber metadata
    char liveliness_keyexpr[512];
    if (create_ros2_liveliness_keyexpr("/turtle1/safe_cmd_vel", "geometry_msgs::msg::dds_::Twist_", 
                                       "4b7b974a54e671b5f16a1829af995a4e", 
                                       "zenoh_subscriber", liveliness_keyexpr, sizeof(liveliness_keyexpr)) != 0) {
        printf("Failed to create liveliness keyexpr\n");
        z_drop(z_move(s));
        return -1;
    }
    
    printf("DEBUG: Declaring liveliness token: %s\n", liveliness_keyexpr);
    
    // Actually declare the liveliness token
    z_owned_liveliness_token_t liveliness_token;
    z_liveliness_token_options_t options;
    z_liveliness_token_options_default(&options);
    
    z_view_keyexpr_t keyexpr_view;
    if (z_view_keyexpr_from_str(&keyexpr_view, liveliness_keyexpr) != Z_OK) {
        printf("Failed to create keyexpr view\n");
        z_drop(z_move(s));
        return -1;
    }
    
    z_result_t result = z_liveliness_declare_token(
        z_session_loan(&s), 
        &liveliness_token,
        z_loan(keyexpr_view),
        &options
    );
    
    if (result != Z_OK) {
        printf("Failed to declare liveliness token (result: %d)\n", result);
        z_drop(z_move(s));
        return -1;
    }
    
    printf("✅ Liveliness token declared successfully!\n");
    
    // Create key expression - use wildcard to catch all messages
    char wildcard_topic[512];
    snprintf(wildcard_topic, sizeof(wildcard_topic), "**");
    
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, wildcard_topic) < 0) {
        printf("❌ Invalid key expression: %s\n", wildcard_topic);
        z_drop(z_move(s));
        return -1;
    }
    
    printf("✅ Key expression created: %s\n", wildcard_topic);
    
    // Start read and lease tasks for zenoh-pico
    if (zp_start_read_task(z_session_loan_mut(&s), NULL) < 0 || 
        zp_start_lease_task(z_session_loan_mut(&s), NULL) < 0) {
        printf("❌ Unable to start read and lease tasks\n");
        z_drop(z_move(s));
        return -1;
    }
    
    // Create subscriber with callback
    z_owned_closure_sample_t callback;
    z_closure_sample(&callback, twist_callback, NULL, NULL);
    
    printf("Declaring Subscriber on '%s'...\n", topic);
    z_owned_subscriber_t sub;
    if (z_declare_subscriber(z_session_loan(&s), &sub, z_view_keyexpr_loan(&ke), z_closure_sample_move(&callback), NULL) < 0) {
        printf("❌ Failed to declare subscriber\n");
        z_drop(z_move(s));
        return -1;
    }
    
    printf("✅ Subscriber declared for topic: %s\n", topic);
    printf("📥 Listening for ROS 2 Twist messages...\n");
    printf("Press Ctrl+C to stop early\n\n");
    
    // Listen for messages
    sleep(duration);
    
    printf("\n🛑 Stopping subscriber...\n");
    
    // Cleanup
    z_subscriber_drop(z_subscriber_move(&sub));
    z_liveliness_undeclare_token(z_liveliness_token_move(&liveliness_token));
    z_session_drop(z_session_move(&s));
    
    printf("✅ Subscriber example completed\n");
    
#else
    printf("❌ Zenoh Pico was compiled without Z_FEATURE_SUBSCRIPTION but this example requires it.\n");
    return -2;
#endif

#ifndef MICROCDR_AVAILABLE
    printf("⚠️  Warning: Micro-CDR not available. CDR deserialization will fail.\n");
#endif

    return 0;
}