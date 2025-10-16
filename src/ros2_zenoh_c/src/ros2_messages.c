/**
 * @file ros2_messages.c
 * @brief Implementation of ROS 2 message serialization/deserialization
 */

#include "ros2_zenoh_c/ros2_messages.h"
#include <string.h>

#ifdef MICROCDR_AVAILABLE
// Helper function to serialize strings
static bool serialize_string(const char* str, ucdrBuffer* buffer) {
    size_t len = strlen(str);
    return ucdr_serialize_uint32_t(buffer, (uint32_t)len) &&
           ucdr_serialize_array_char(buffer, (const char*)str, len);
}

// Helper function to deserialize strings
static bool deserialize_string(char* str, size_t max_len, ucdrBuffer* buffer) {
    uint32_t len;
    if (!ucdr_deserialize_uint32_t(buffer, &len)) {
        return false;
    }
    
    if (len >= max_len) {
        return false;  // String too long
    }
    
    return ucdr_deserialize_array_char(buffer, str, len);
}

// Time serialization
bool ros2_time_serialize(const ros2_time_t* msg, ucdrBuffer* buffer) {
    return ucdr_serialize_int32_t(buffer, msg->sec) &&
           ucdr_serialize_uint32_t(buffer, msg->nanosec);
}

bool ros2_time_deserialize(ros2_time_t* msg, ucdrBuffer* buffer) {
    return ucdr_deserialize_int32_t(buffer, &msg->sec) &&
           ucdr_deserialize_uint32_t(buffer, &msg->nanosec);
}

// Duration serialization
bool ros2_duration_serialize(const ros2_duration_t* msg, ucdrBuffer* buffer) {
    return ucdr_serialize_int32_t(buffer, msg->sec) &&
           ucdr_serialize_uint32_t(buffer, msg->nanosec);
}

bool ros2_duration_deserialize(ros2_duration_t* msg, ucdrBuffer* buffer) {
    return ucdr_deserialize_int32_t(buffer, &msg->sec) &&
           ucdr_deserialize_uint32_t(buffer, &msg->nanosec);
}

// Header serialization
bool ros2_header_serialize(const ros2_header_t* msg, ucdrBuffer* buffer) {
    return ros2_time_serialize(&msg->stamp, buffer) &&
           serialize_string(msg->frame_id, buffer);
}

bool ros2_header_deserialize(ros2_header_t* msg, ucdrBuffer* buffer) {
    return ros2_time_deserialize(&msg->stamp, buffer) &&
           deserialize_string(msg->frame_id, sizeof(msg->frame_id), buffer);
}

// Vector3 serialization
bool ros2_vector3_serialize(const ros2_vector3_t* msg, ucdrBuffer* buffer) {
    return ucdr_serialize_float(buffer, msg->x) &&
           ucdr_serialize_float(buffer, msg->y) &&
           ucdr_serialize_float(buffer, msg->z);
}

bool ros2_vector3_deserialize(ros2_vector3_t* msg, ucdrBuffer* buffer) {
    return ucdr_deserialize_float(buffer, &msg->x) &&
           ucdr_deserialize_float(buffer, &msg->y) &&
           ucdr_deserialize_float(buffer, &msg->z);
}

// Quaternion serialization
bool ros2_quaternion_serialize(const ros2_quaternion_t* msg, ucdrBuffer* buffer) {
    return ucdr_serialize_float(buffer, msg->x) &&
           ucdr_serialize_float(buffer, msg->y) &&
           ucdr_serialize_float(buffer, msg->z) &&
           ucdr_serialize_float(buffer, msg->w);
}

bool ros2_quaternion_deserialize(ros2_quaternion_t* msg, ucdrBuffer* buffer) {
    return ucdr_deserialize_float(buffer, &msg->x) &&
           ucdr_deserialize_float(buffer, &msg->y) &&
           ucdr_deserialize_float(buffer, &msg->z) &&
           ucdr_deserialize_float(buffer, &msg->w);
}

// Transform serialization
bool ros2_transform_serialize(const ros2_transform_t* msg, ucdrBuffer* buffer) {
    return ros2_vector3_serialize(&msg->translation, buffer) &&
           ros2_quaternion_serialize(&msg->rotation, buffer);
}

bool ros2_transform_deserialize(ros2_transform_t* msg, ucdrBuffer* buffer) {
    return ros2_vector3_deserialize(&msg->translation, buffer) &&
           ros2_quaternion_deserialize(&msg->rotation, buffer);
}

// TransformStamped serialization
bool ros2_transform_stamped_serialize(const ros2_transform_stamped_t* msg, ucdrBuffer* buffer) {
    return ros2_header_serialize(&msg->header, buffer) &&
           ros2_transform_serialize(&msg->transform, buffer) &&
           serialize_string(msg->child_frame_id, buffer);
}

bool ros2_transform_stamped_deserialize(ros2_transform_stamped_t* msg, ucdrBuffer* buffer) {
    return ros2_header_deserialize(&msg->header, buffer) &&
           ros2_transform_deserialize(&msg->transform, buffer) &&
           deserialize_string(msg->child_frame_id, sizeof(msg->child_frame_id), buffer);
}

// Twist serialization
bool ros2_twist_serialize(const ros2_twist_t* msg, ucdrBuffer* buffer) {
    return ros2_vector3_serialize(&msg->linear, buffer) &&
           ros2_vector3_serialize(&msg->angular, buffer);
}

bool ros2_twist_deserialize(ros2_twist_t* msg, ucdrBuffer* buffer) {
    return ros2_vector3_deserialize(&msg->linear, buffer) &&
           ros2_vector3_deserialize(&msg->angular, buffer);
}

// TwistStamped serialization
bool ros2_twist_stamped_serialize(const ros2_twist_stamped_t* msg, ucdrBuffer* buffer) {
    return ros2_header_serialize(&msg->header, buffer) &&
           ros2_twist_serialize(&msg->twist, buffer);
}

bool ros2_twist_stamped_deserialize(ros2_twist_stamped_t* msg, ucdrBuffer* buffer) {
    return ros2_header_deserialize(&msg->header, buffer) &&
           ros2_twist_deserialize(&msg->twist, buffer);
}

// Pose serialization
bool ros2_pose_serialize(const ros2_pose_t* msg, ucdrBuffer* buffer) {
    return ros2_vector3_serialize(&msg->position, buffer) &&
           ros2_quaternion_serialize(&msg->orientation, buffer);
}

bool ros2_pose_deserialize(ros2_pose_t* msg, ucdrBuffer* buffer) {
    return ros2_vector3_deserialize(&msg->position, buffer) &&
           ros2_quaternion_deserialize(&msg->orientation, buffer);
}

// PoseStamped serialization
bool ros2_pose_stamped_serialize(const ros2_pose_stamped_t* msg, ucdrBuffer* buffer) {
    return ros2_header_serialize(&msg->header, buffer) &&
           ros2_pose_serialize(&msg->pose, buffer);
}

bool ros2_pose_stamped_deserialize(ros2_pose_stamped_t* msg, ucdrBuffer* buffer) {
    return ros2_header_deserialize(&msg->header, buffer) &&
           ros2_pose_deserialize(&msg->pose, buffer);
}

// Point serialization
bool ros2_point_serialize(const ros2_point_t* msg, ucdrBuffer* buffer) {
    return ucdr_serialize_float(buffer, msg->x) &&
           ucdr_serialize_float(buffer, msg->y) &&
           ucdr_serialize_float(buffer, msg->z);
}

bool ros2_point_deserialize(ros2_point_t* msg, ucdrBuffer* buffer) {
    return ucdr_deserialize_float(buffer, &msg->x) &&
           ucdr_deserialize_float(buffer, &msg->y) &&
           ucdr_deserialize_float(buffer, &msg->z);
}

// PointStamped serialization
bool ros2_point_stamped_serialize(const ros2_point_stamped_t* msg, ucdrBuffer* buffer) {
    return ros2_header_serialize(&msg->header, buffer) &&
           ros2_point_serialize(&msg->point, buffer);
}

bool ros2_point_stamped_deserialize(ros2_point_stamped_t* msg, ucdrBuffer* buffer) {
    return ros2_header_deserialize(&msg->header, buffer) &&
           ros2_point_deserialize(&msg->point, buffer);
}

#endif // MICROCDR_AVAILABLE
