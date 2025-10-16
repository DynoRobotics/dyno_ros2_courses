/**
 * @file ros2_messages.h
 * @brief ROS 2 message type definitions for C
 * 
 * This file contains the C struct definitions for common ROS 2 message types
 * compatible with Micro-CDR serialization.
 */

#ifndef ROS2_MESSAGES_H
#define ROS2_MESSAGES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef MICROCDR_AVAILABLE
#include <ucdr/microcdr.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Built-in interfaces
typedef struct {
    int32_t sec;
    uint32_t nanosec;
} ros2_time_t;

typedef struct {
    int32_t sec;
    uint32_t nanosec;
} ros2_duration_t;

// Standard messages
typedef struct {
    ros2_time_t stamp;
    char frame_id[256];  // Fixed size for simplicity
} ros2_header_t;

typedef struct {
    float x;
    float y;
    float z;
} ros2_vector3_t;

typedef struct {
    float x;
    float y;
    float z;
    float w;
} ros2_quaternion_t;

typedef struct {
    ros2_vector3_t translation;
    ros2_quaternion_t rotation;
} ros2_transform_t;

typedef struct {
    ros2_header_t header;
    ros2_transform_t transform;
    char child_frame_id[256];
} ros2_transform_stamped_t;

// Geometry messages
typedef struct {
    ros2_vector3_t linear;
    ros2_vector3_t angular;
} ros2_twist_t;

typedef struct {
    ros2_header_t header;
    ros2_twist_t twist;
} ros2_twist_stamped_t;

typedef struct {
    ros2_vector3_t position;
    ros2_quaternion_t orientation;
} ros2_pose_t;

typedef struct {
    ros2_header_t header;
    ros2_pose_t pose;
} ros2_pose_stamped_t;

typedef struct {
    float x;
    float y;
    float z;
} ros2_point_t;

typedef struct {
    ros2_header_t header;
    ros2_point_t point;
} ros2_point_stamped_t;

// Message type hashes (RIHS01 format)
#define ROS2_TIME_TYPE_HASH "RIHS01_717e0f25a0eeb467efd904dceb35d203"
#define ROS2_DURATION_TYPE_HASH "RIHS01_3d408c0c625e7d1207004a7a515d89f"
#define ROS2_HEADER_TYPE_HASH "RIHS01_f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01"
#define ROS2_VECTOR3_TYPE_HASH "RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d"
#define ROS2_QUATERNION_TYPE_HASH "RIHS01_a779bccf38b231c1e971d39c8647d840e6d3c0c6c42dab46084ad5ec8875b6c"
#define ROS2_TRANSFORM_TYPE_HASH "RIHS01_ac9eff28ab6bf886b7e19262f9a5c5b6"
#define ROS2_TRANSFORM_STAMPED_TYPE_HASH "RIHS01_b576c97df95e8ea178b98b77e3e73683"
#define ROS2_TWIST_TYPE_HASH "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a"
#define ROS2_TWIST_STAMPED_TYPE_HASH "RIHS01_98d195b7b237edc0c2e4d9bc2c659277"
#define ROS2_POSE_TYPE_HASH "RIHS01_e56d19e010e5da1c709d656f2b8c6e57"
#define ROS2_POSE_STAMPED_TYPE_HASH "RIHS01_98d4c95c83a051a264e3cd509a7052c0"
#define ROS2_POINT_TYPE_HASH "RIHS01_4a842b65f413084b992483b2eafc991c"
#define ROS2_POINT_STAMPED_TYPE_HASH "RIHS01_c63a13b9f1201834b6c22e7552a316b4"

// Serialization functions for each message type
#ifdef MICROCDR_AVAILABLE
bool ros2_time_serialize(const ros2_time_t* msg, ucdrBuffer* buffer);
bool ros2_time_deserialize(ros2_time_t* msg, ucdrBuffer* buffer);

bool ros2_duration_serialize(const ros2_duration_t* msg, ucdrBuffer* buffer);
bool ros2_duration_deserialize(ros2_duration_t* msg, ucdrBuffer* buffer);

bool ros2_header_serialize(const ros2_header_t* msg, ucdrBuffer* buffer);
bool ros2_header_deserialize(ros2_header_t* msg, ucdrBuffer* buffer);

bool ros2_vector3_serialize(const ros2_vector3_t* msg, ucdrBuffer* buffer);
bool ros2_vector3_deserialize(ros2_vector3_t* msg, ucdrBuffer* buffer);

bool ros2_quaternion_serialize(const ros2_quaternion_t* msg, ucdrBuffer* buffer);
bool ros2_quaternion_deserialize(ros2_quaternion_t* msg, ucdrBuffer* buffer);

bool ros2_transform_serialize(const ros2_transform_t* msg, ucdrBuffer* buffer);
bool ros2_transform_deserialize(ros2_transform_t* msg, ucdrBuffer* buffer);

bool ros2_transform_stamped_serialize(const ros2_transform_stamped_t* msg, ucdrBuffer* buffer);
bool ros2_transform_stamped_deserialize(ros2_transform_stamped_t* msg, ucdrBuffer* buffer);

bool ros2_twist_serialize(const ros2_twist_t* msg, ucdrBuffer* buffer);
bool ros2_twist_deserialize(ros2_twist_t* msg, ucdrBuffer* buffer);

bool ros2_twist_stamped_serialize(const ros2_twist_stamped_t* msg, ucdrBuffer* buffer);
bool ros2_twist_stamped_deserialize(ros2_twist_stamped_t* msg, ucdrBuffer* buffer);

bool ros2_pose_serialize(const ros2_pose_t* msg, ucdrBuffer* buffer);
bool ros2_pose_deserialize(ros2_pose_t* msg, ucdrBuffer* buffer);

bool ros2_pose_stamped_serialize(const ros2_pose_stamped_t* msg, ucdrBuffer* buffer);
bool ros2_pose_stamped_deserialize(ros2_pose_stamped_t* msg, ucdrBuffer* buffer);

bool ros2_point_serialize(const ros2_point_t* msg, ucdrBuffer* buffer);
bool ros2_point_deserialize(ros2_point_t* msg, ucdrBuffer* buffer);

bool ros2_point_stamped_serialize(const ros2_point_stamped_t* msg, ucdrBuffer* buffer);
bool ros2_point_stamped_deserialize(ros2_point_stamped_t* msg, ucdrBuffer* buffer);
#endif

#ifdef __cplusplus
}
#endif

#endif // ROS2_MESSAGES_H
