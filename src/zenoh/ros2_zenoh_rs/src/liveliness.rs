/// Liveliness token management for ROS 2 metadata discovery

use anyhow::Result;
use std::sync::Arc;
use zenoh::Session;
use zenoh::Wait;

/// Manage liveliness tokens for ROS 2 publishers and subscribers
pub struct LivelinessManager {
    session: Arc<Session>,
}

impl LivelinessManager {
    pub fn new(session: Arc<Session>) -> Self {
        Self { session }
    }

    /// Declare publisher liveliness token
    pub fn declare_publisher_token(
        &self,
        topic_name: &str,
        message_type: &str,
        type_hash: &str,
        node_name: &str,
        node_namespace: &str,
    ) -> Result<zenoh::liveliness::LivelinessToken> {
        let key = self.build_liveliness_key(
            "MP",
            topic_name,
            message_type,
            type_hash,
            node_name,
            node_namespace,
        );
        
        Ok(self.session.liveliness().declare_token(&key).wait().map_err(|e| anyhow::anyhow!("{}", e))?)
    }

    /// Declare subscriber liveliness token
    pub fn declare_subscriber_token(
        &self,
        topic_name: &str,
        message_type: &str,
        type_hash: &str,
        node_name: &str,
        node_namespace: &str,
    ) -> Result<zenoh::liveliness::LivelinessToken> {
        let key = self.build_liveliness_key(
            "MS",
            topic_name,
            message_type,
            type_hash,
            node_name,
            node_namespace,
        );
        
        Ok(self.session.liveliness().declare_token(&key).wait().map_err(|e| anyhow::anyhow!("{}", e))?)
    }

    /// Build liveliness key in ROS 2 format
    fn build_liveliness_key(
        &self,
        entity_type: &str, // "MP" for publisher, "MS" for subscriber
        topic_name: &str,
        message_type: &str,
        type_hash: &str,
        node_name: &str,
        node_namespace: &str,
    ) -> String {
        let topic_part = topic_name.trim_start_matches('/');
        let namespace_part = if node_namespace.is_empty() || node_namespace == "/" {
            ""
        } else {
            node_namespace.trim_matches('/')
        };

        // Format: @ros2_lv/<zenoh_id>/<entity_type>/<namespace>/<node_name>/<topic>/<type>/<hash>
        // Simplified without zenoh_id for now
        format!(
            "@ros2_lv/{}/{}/{}/{}/{}/{}",
            entity_type,
            if namespace_part.is_empty() {
                "_"
            } else {
                namespace_part
            },
            node_name,
            topic_part,
            message_type.replace("/", "::"),
            type_hash
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_liveliness_key_format() {
        // We can't easily test with a real session, but we can test key building logic
        let manager = LivelinessManager {
            session: Arc::new(unsafe { std::mem::zeroed() }),
        };

        let key = manager.build_liveliness_key(
            "MP",
            "/turtle1/cmd_vel",
            "geometry_msgs/msg/Twist",
            "RIHS01_abc123",
            "test_node",
            "/test_ns",
        );

        assert!(key.contains("@ros2_lv/"));
        assert!(key.contains("MP"));
        assert!(key.contains("test_node"));
        assert!(key.contains("turtle1/cmd_vel"));
    }
}

