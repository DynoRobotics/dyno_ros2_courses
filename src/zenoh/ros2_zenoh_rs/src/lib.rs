/// ROS 2 over Zenoh transport layer in Rust
/// 
/// This library provides both raw byte-based APIs (for NAPI bindings)
/// and typed APIs (for pure Rust usage).

pub mod attachments;
pub mod liveliness;
pub mod node;
pub mod raw;
pub mod types;

// Re-export main types
pub use node::Node;
pub use raw::{RawPublisher, RawSubscriber};
pub use types::TypeRegistry;

pub use anyhow::{Error, Result};

