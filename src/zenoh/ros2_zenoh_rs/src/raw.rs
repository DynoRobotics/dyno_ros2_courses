/// Raw publisher and subscriber (byte-based API for NAPI layer)

use anyhow::Result;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use zenoh::bytes::Encoding;
use zenoh::Session;
use zenoh::Wait;

use crate::attachments::build_attachment;
use crate::liveliness::LivelinessManager;
use crate::types::TypeRegistry;

/// Raw publisher that accepts pre-serialized bytes
pub struct RawPublisher {
    session: Arc<Session>,
    dds_key: String,
    publisher_gid: [u8; 16],
    sequence_number: AtomicU64,
    _liveliness_token: zenoh::liveliness::LivelinessToken,
}

impl RawPublisher {
    pub fn new(
        session: Arc<Session>,
        topic: &str,
        msg_type: &str,
        node_name: &str,
        node_namespace: &str,
    ) -> Result<Self> {
        let type_registry = TypeRegistry::new();
        let dds_key = type_registry.create_dds_key(topic, msg_type);

        // Generate random publisher GID
        let publisher_gid: [u8; 16] = rand::random();

        // Declare liveliness token
        let liveliness_manager = LivelinessManager::new(session.clone());
        let type_hash = type_registry.get_type_hash(msg_type);
        let dds_type = type_registry.get_dds_type(msg_type);
        let liveliness_token = liveliness_manager.declare_publisher_token(
            topic,
            &dds_type,
            &type_hash,
            node_name,
            node_namespace,
        )?;

        Ok(Self {
            session,
            dds_key,
            publisher_gid,
            sequence_number: AtomicU64::new(0),
            _liveliness_token: liveliness_token,
        })
    }

    /// Publish pre-serialized CDR bytes
    pub fn publish(&self, cdr_data: &[u8]) -> Result<()> {
        let seq = self.sequence_number.fetch_add(1, Ordering::SeqCst);
        let attachment = build_attachment(seq, &self.publisher_gid);

        self.session
            .put(&self.dds_key, cdr_data)
            .encoding(Encoding::ZENOH_BYTES)
            .attachment(attachment)
            .wait()
            .map_err(|e| anyhow::anyhow!("{}", e))?;

        Ok(())
    }

    pub fn topic(&self) -> &str {
        &self.dds_key
    }
}

/// Raw subscriber that delivers raw bytes to callback
pub struct RawSubscriber {
    _subscriber: zenoh::pubsub::Subscriber<()>,
    _liveliness_token: zenoh::liveliness::LivelinessToken,
}

impl RawSubscriber {
    pub fn new<F>(
        session: Arc<Session>,
        topic: &str,
        msg_type: &str,
        node_name: &str,
        node_namespace: &str,
        callback: F,
    ) -> Result<Self>
    where
        F: Fn(&[u8]) + Send + Sync + 'static,
    {
        let type_registry = TypeRegistry::new();
        let dds_key = type_registry.create_dds_key(topic, msg_type);

        // Declare liveliness token
        let liveliness_manager = LivelinessManager::new(session.clone());
        let type_hash = type_registry.get_type_hash(msg_type);
        let dds_type = type_registry.get_dds_type(msg_type);
        let liveliness_token = liveliness_manager.declare_subscriber_token(
            topic,
            &dds_type,
            &type_hash,
            node_name,
            node_namespace,
        )?;

        // Create subscriber
        let subscriber = session
            .declare_subscriber(&dds_key)
            .callback(move |sample| {
                let payload = sample.payload().to_bytes();
                callback(&payload);
            })
            .wait()
            .map_err(|e| anyhow::anyhow!("{}", e))?;

        Ok(Self {
            _subscriber: subscriber,
            _liveliness_token: liveliness_token,
        })
    }
}

