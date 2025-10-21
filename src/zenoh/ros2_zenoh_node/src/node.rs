/// NAPI bindings for ROS 2 Node

use napi::bindgen_prelude::*;
use napi_derive::napi;
use std::sync::Arc;

use crate::publisher::NativePublisher;
use crate::subscriber::NativeSubscriber;

#[napi]
pub struct NativeNode {
    inner: Arc<ros2_zenoh_rs::Node>,
}

#[napi]
impl NativeNode {
    /// Create a new ROS 2 node
    #[napi(constructor)]
    pub fn new(name: String, namespace: Option<String>) -> Result<Self> {
        let ns = namespace.unwrap_or_else(|| "/".to_string());
        let inner = ros2_zenoh_rs::Node::new(&name, &ns)
            .map_err(|e| Error::from_reason(format!("Failed to create node: {}", e)))?;
        
        Ok(Self {
            inner: Arc::new(inner),
        })
    }

    /// Get node name
    #[napi]
    pub fn get_name(&self) -> String {
        self.inner.name().to_string()
    }

    /// Get node namespace
    #[napi]
    pub fn get_namespace(&self) -> String {
        self.inner.namespace().to_string()
    }

    /// Create a raw publisher (accepts Buffer with CDR bytes)
    #[napi]
    pub fn create_raw_publisher(&self, topic: String, msg_type: String) -> Result<NativePublisher> {
        NativePublisher::new(self.inner.clone(), topic, msg_type)
    }

    /// Create a raw subscriber (delivers Buffer with CDR bytes to callback)
    #[napi]
    pub fn create_raw_subscriber(
        &self,
        topic: String,
        msg_type: String,
        callback: JsFunction,
    ) -> Result<NativeSubscriber> {
        NativeSubscriber::new(self.inner.clone(), topic, msg_type, callback)
    }
}

