/// Type definitions and utilities for ROS 2 DDS interop

use std::collections::HashMap;

/// Type hash mappings for common ROS 2 message types
pub struct TypeRegistry {
    type_hashes: HashMap<String, String>,
    dds_types: HashMap<String, String>,
}

impl TypeRegistry {
    pub fn new() -> Self {
        let mut type_hashes = HashMap::new();
        let mut dds_types = HashMap::new();

        // Common message type hashes (from Python implementation)
        type_hashes.insert(
            "geometry_msgs/msg/Twist".to_string(),
            "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a".to_string(),
        );
        type_hashes.insert(
            "geometry_msgs/msg/Vector3".to_string(),
            "RIHS01_4a7b354a29a8a324c9f9ce904d36969a1eb5b805c515e434cbabac4562cb363d".to_string(),
        );
        type_hashes.insert(
            "builtin_interfaces/msg/Time".to_string(),
            "RIHS01_4a7b354a29a8a324c9f9ce904d36969a1eb5b805c515e434cbabac4562cb363d".to_string(),
        );
        type_hashes.insert(
            "std_msgs/msg/Header".to_string(),
            "RIHS01_4a7b354a29a8a324c9f9ce904d36969a1eb5b805c515e434cbabac4562cb363d".to_string(),
        );

        // DDS type mappings
        dds_types.insert(
            "geometry_msgs/msg/Twist".to_string(),
            "geometry_msgs::msg::dds_::Twist_".to_string(),
        );
        dds_types.insert(
            "geometry_msgs/msg/Vector3".to_string(),
            "geometry_msgs::msg::dds_::Vector3_".to_string(),
        );
        dds_types.insert(
            "builtin_interfaces/msg/Time".to_string(),
            "builtin_interfaces::msg::dds_::Time_".to_string(),
        );
        dds_types.insert(
            "std_msgs/msg/Header".to_string(),
            "std_msgs::msg::dds_::Header_".to_string(),
        );

        Self {
            type_hashes,
            dds_types,
        }
    }

    pub fn get_type_hash(&self, msg_type: &str) -> String {
        self.type_hashes
            .get(msg_type)
            .cloned()
            .unwrap_or_else(|| format!("RIHS01_{}", "0".repeat(64)))
    }

    pub fn get_dds_type(&self, msg_type: &str) -> String {
        self.dds_types
            .get(msg_type)
            .cloned()
            .unwrap_or_else(|| format!("{}::dds_", msg_type.replace("/", "::")))
    }

    /// Create DDS interop key for a topic
    /// Format: "0/<topic>/<dds_type>/<type_hash>"
    pub fn create_dds_key(&self, topic: &str, msg_type: &str) -> String {
        let topic_part = topic.trim_start_matches('/');
        let dds_type = self.get_dds_type(msg_type);
        let type_hash = self.get_type_hash(msg_type);
        format!("0/{}/{}/{}", topic_part, dds_type, type_hash)
    }
}

impl Default for TypeRegistry {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dds_key_generation() {
        let registry = TypeRegistry::new();
        let key = registry.create_dds_key("/turtle1/cmd_vel", "geometry_msgs/msg/Twist");
        assert!(key.starts_with("0/turtle1/cmd_vel/"));
        assert!(key.contains("geometry_msgs::msg::dds_::Twist_"));
        assert!(key.contains("RIHS01_"));
    }
}

