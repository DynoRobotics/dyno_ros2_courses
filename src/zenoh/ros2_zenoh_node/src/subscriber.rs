/// NAPI bindings for ROS 2 Subscriber

use napi::bindgen_prelude::*;
use napi::threadsafe_function::{ThreadsafeFunction, ThreadsafeFunctionCallMode, ThreadSafeCallContext};
use napi_derive::napi;
use std::sync::Arc;

#[napi]
pub struct NativeSubscriber {
    _inner: ros2_zenoh_rs::raw::RawSubscriber,
}

impl NativeSubscriber {
    pub(crate) fn new(
        node: Arc<ros2_zenoh_rs::Node>,
        topic: String,
        msg_type: String,
        callback: JsFunction,
    ) -> Result<Self> {
        // Create a thread-safe function to call back into JavaScript
        let tsfn: ThreadsafeFunction<Vec<u8>> = callback
            .create_threadsafe_function(0, |ctx: ThreadSafeCallContext<Vec<u8>>| {
                // Convert Vec<u8> to Buffer for JavaScript
                // The callback will be called with just the buffer as the argument
                ctx.env.create_buffer_with_data(ctx.value).map(|b| vec![b.into_raw()])
            })?;

        // Create the subscriber with the callback
        let inner = node
            .create_raw_subscriber(&topic, &msg_type, move |data: &[u8]| {
                // Call the JavaScript callback with the data
                let data_vec = data.to_vec();
                let _ = tsfn.call(Ok(data_vec), ThreadsafeFunctionCallMode::NonBlocking);
            })
            .map_err(|e| Error::from_reason(format!("Failed to create subscriber: {}", e)))?;

        Ok(Self { _inner: inner })
    }
}

