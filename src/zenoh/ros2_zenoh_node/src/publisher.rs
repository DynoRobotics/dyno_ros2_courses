/// NAPI bindings for ROS 2 Publisher

use napi::bindgen_prelude::*;
use napi_derive::napi;
use std::sync::Arc;

#[napi]
pub struct NativePublisher {
    inner: ros2_zenoh_rs::raw::RawPublisher,
}

impl NativePublisher {
    pub(crate) fn new(
        node: Arc<ros2_zenoh_rs::Node>,
        topic: String,
        msg_type: String,
    ) -> Result<Self> {
        let inner = node
            .create_raw_publisher(&topic, &msg_type)
            .map_err(|e| Error::from_reason(format!("Failed to create publisher: {}", e)))?;
        
        Ok(Self { inner })
    }
}

#[napi]
impl NativePublisher {
    /// Publish pre-serialized CDR bytes
    #[napi]
    pub fn publish_raw(&self, data: Buffer) -> Result<()> {
        self.inner
            .publish(data.as_ref())
            .map_err(|e| Error::from_reason(format!("Failed to publish: {}", e)))
    }

    /// Get the topic this publisher is publishing to
    #[napi]
    pub fn get_topic(&self) -> String {
        self.inner.topic().to_string()
    }
}

