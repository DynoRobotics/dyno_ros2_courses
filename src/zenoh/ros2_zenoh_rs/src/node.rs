/// ROS 2 Node abstraction

use anyhow::Result;
use std::sync::Arc;
use zenoh::Session;
use zenoh::Wait;

use crate::raw::{RawPublisher, RawSubscriber};

/// ROS 2 Node managing Zenoh session and resources
pub struct Node {
    name: String,
    namespace: String,
    session: Arc<Session>,
}

impl Node {
    /// Create a new node with the given name and namespace
    pub fn new(name: &str, namespace: &str) -> Result<Self> {
        let config = zenoh::Config::default();
        let session = Arc::new(zenoh::open(config).wait().map_err(|e| anyhow::anyhow!("{}", e))?);

        Ok(Self {
            name: name.to_string(),
            namespace: namespace.to_string(),
            session,
        })
    }

    /// Create a new node with custom Zenoh config
    pub fn with_config(name: &str, namespace: &str, config: zenoh::Config) -> Result<Self> {
        let session = Arc::new(zenoh::open(config).wait().map_err(|e| anyhow::anyhow!("{}", e))?);

        Ok(Self {
            name: name.to_string(),
            namespace: namespace.to_string(),
            session,
        })
    }

    /// Create a raw publisher (accepts pre-serialized bytes)
    pub fn create_raw_publisher(&self, topic: &str, msg_type: &str) -> Result<RawPublisher> {
        RawPublisher::new(
            self.session.clone(),
            topic,
            msg_type,
            &self.name,
            &self.namespace,
        )
    }

    /// Create a raw subscriber (delivers raw bytes to callback)
    pub fn create_raw_subscriber<F>(
        &self,
        topic: &str,
        msg_type: &str,
        callback: F,
    ) -> Result<RawSubscriber>
    where
        F: Fn(&[u8]) + Send + Sync + 'static,
    {
        RawSubscriber::new(
            self.session.clone(),
            topic,
            msg_type,
            &self.name,
            &self.namespace,
            callback,
        )
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn namespace(&self) -> &str {
        &self.namespace
    }

    pub fn session(&self) -> &Arc<Session> {
        &self.session
    }
}

