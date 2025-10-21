/// Attachment handling for ROS 2 metadata
/// Format: seq (8 bytes) + timestamp (8 bytes) + VarInt(16) + GID (16 bytes)

use std::time::{SystemTime, UNIX_EPOCH};

/// Build attachment for a message
/// Version 3 format: seq + timestamp + VarInt(16) + GID
pub fn build_attachment(sequence: u64, publisher_gid: &[u8; 16]) -> Vec<u8> {
    let timestamp_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;

    let mut attachment = Vec::with_capacity(33);
    
    // Sequence number (8 bytes, little-endian)
    attachment.extend_from_slice(&sequence.to_le_bytes());
    
    // Timestamp in nanoseconds (8 bytes, little-endian)
    attachment.extend_from_slice(&timestamp_ns.to_le_bytes());
    
    // VarInt(16) - single byte 0x10
    attachment.push(0x10);
    
    // Publisher GID (16 bytes)
    attachment.extend_from_slice(publisher_gid);
    
    attachment
}

/// Parse attachment from received message
pub struct AttachmentData {
    pub sequence: Option<u64>,
    pub timestamp_ns: Option<u64>,
    pub gid: Option<[u8; 16]>,
}

pub fn parse_attachment(data: &[u8]) -> AttachmentData {
    if data.len() < 33 {
        return AttachmentData {
            sequence: None,
            timestamp_ns: None,
            gid: None,
        };
    }

    // Parse sequence (first 8 bytes)
    let sequence = u64::from_le_bytes(data[0..8].try_into().unwrap());
    
    // Parse timestamp (next 8 bytes)
    let timestamp_ns = u64::from_le_bytes(data[8..16].try_into().unwrap());
    
    // Skip VarInt byte at position 16
    // Parse GID (last 16 bytes)
    let mut gid = [0u8; 16];
    gid.copy_from_slice(&data[17..33]);

    AttachmentData {
        sequence: Some(sequence),
        timestamp_ns: Some(timestamp_ns),
        gid: Some(gid),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_attachment_round_trip() {
        let gid = [1u8; 16];
        let seq = 42;
        
        let attachment = build_attachment(seq, &gid);
        assert_eq!(attachment.len(), 33);
        
        let parsed = parse_attachment(&attachment);
        assert_eq!(parsed.sequence, Some(seq));
        assert!(parsed.timestamp_ns.is_some());
        assert_eq!(parsed.gid, Some(gid));
    }
}

