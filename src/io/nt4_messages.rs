use serde::{Deserialize, Serialize};

/// NT4 control messages are JSON arrays sent on the WebSocket text channel.
/// Each element of the outer array is one message; we always send/receive one
/// at a time so this wrapper is just `[msg]`.
///
/// Spec: ntcore/doc/networktables4.adoc — we only model the message types
/// we actually need (subscribe + announce + unannounce). Others are ignored.

#[derive(Debug, Serialize)]
#[serde(tag = "method", content = "params")]
pub enum ClientMessage {
    #[serde(rename = "subscribe")]
    Subscribe(SubscribeParams),
}

#[derive(Debug, Serialize)]
pub struct SubscribeParams {
    pub topics:    Vec<String>,
    pub subuid:    u32,
    pub options:   SubscribeOptions,
}

#[derive(Debug, Serialize, Default)]
pub struct SubscribeOptions {
    /// If true, the server treats `topics` as glob prefixes and we get
    /// every topic that matches. We use this with `[""]` to get *all* topics.
    pub prefix:    bool,
    /// Server-side throttling, in microseconds. 0 = send every value.
    /// We pick a small but non-zero value (10ms) so the firehose doesn't
    /// overwhelm us if a topic is publishing at 1 kHz.
    pub periodic:  f64,
    /// We don't care about local-only topics or all-history.
    pub all:       bool,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "method", content = "params")]
pub enum ServerMessage {
    #[serde(rename = "announce")]
    Announce(AnnounceParams),
    #[serde(rename = "unannounce")]
    Unannounce(UnannounceParams),
    /// We ignore other server messages (properties, etc.) but they shouldn't
    /// fail the parser — serde with #[serde(other)] would handle that, but
    /// it's annoying with content-tagged enums. We use a permissive top-level
    /// parser instead (see `parse_server_messages` below).
    #[serde(other)]
    Other,
}

#[derive(Debug, Deserialize)]
pub struct AnnounceParams {
    pub name:        String,
    /// Stable numeric ID for this topic, assigned by the server.
    /// Binary value frames reference topics by id, not name, so we maintain
    /// an id→name map on our side.
    pub id:          u32,
    /// NT4 type string: "double", "int", "boolean", "string", "double[]", etc.
    /// We only handle the scalar types — array types get logged and ignored.
    #[serde(rename = "type")]
    pub type_str:    String,
}

#[derive(Debug, Deserialize)]
pub struct UnannounceParams {
    pub name: String,
    pub id:   u32,
}

/// Parse a JSON text frame containing zero or more NT4 server messages.
/// Unknown message types are silently dropped — see the `Other` variant
/// above. We do this rather than failing because the NT4 spec is allowed
/// to grow new message types and we want to remain forward-compatible.
pub fn parse_server_messages(text: &str) -> Vec<ServerMessage> {
    serde_json::from_str::<Vec<ServerMessage>>(text).unwrap_or_default()
}

/// A single value update parsed from a binary MessagePack frame.
/// Binary frames have the structure: [topic_id, timestamp_us, type_id, value]
/// — a 4-element MsgPack array.
#[derive(Debug, Clone)]
pub struct ValueUpdate {
    pub topic_id:     u32,
    pub timestamp_us: u64,
    pub value:        Value,
}

/// The supported value types. NT4's type-id integers map to these.
/// Type ids: 1=double, 2=int64, 3=float, 4=string, 5=boolean.
#[derive(Debug, Clone)]
pub enum Value {
    Double(f64),
    Int64(i64),
    Float(f32),
    Boolean(bool),
    StringVal(String),
}

/// Parse one or more value updates from a binary WebSocket frame. NT4
/// concatenates multiple 4-element arrays into a single binary frame, so
/// we keep reading until we run out of bytes.
pub fn parse_binary_frame(bytes: &[u8]) -> Vec<ValueUpdate> {
    let mut cursor = std::io::Cursor::new(bytes);
    let mut out = Vec::new();
    while (cursor.position() as usize) < bytes.len() {
        match rmpv::decode::read_value(&mut cursor) {
            Ok(rmpv::Value::Array(arr)) if arr.len() == 4 => {
                let Some(topic_id)     = arr[0].as_u64().map(|v| v as u32) else { continue };
                let Some(timestamp_us) = arr[1].as_u64() else { continue };
                let Some(type_id)      = arr[2].as_u64() else { continue };
                let Some(value)        = decode_value(type_id, &arr[3]) else { continue };
                out.push(ValueUpdate { topic_id, timestamp_us, value });
            }
            Ok(_) => {
                // Not a 4-element array — malformed, skip it. Cursor has
                // already been advanced past the bad value.
            }
            Err(_) => break,
        }
    }
    out
}

fn decode_value(type_id: u64, v: &rmpv::Value) -> Option<Value> {
    match type_id {
        1 => v.as_f64().map(Value::Double),
        2 => v.as_i64().map(Value::Int64),
        3 => v.as_f64().map(|f| Value::Float(f as f32)),
        4 => v.as_str().map(|s| Value::StringVal(s.to_string())),
        5 => v.as_bool().map(Value::Boolean),
        _ => None, // arrays, raw bytes, etc. — not handled yet
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_announce() {
        let text = r#"[{"method":"announce","params":{"name":"/foo","id":7,"type":"double","pubuid":2,"properties":{}}}]"#;
        let msgs = parse_server_messages(text);
        assert_eq!(msgs.len(), 1);
        match &msgs[0] {
            ServerMessage::Announce(a) => {
                assert_eq!(a.id, 7);
                assert_eq!(a.name, "/foo");
                assert_eq!(a.type_str, "double");
            }
            _ => panic!("expected announce"),
        }
    }

    #[test]
    fn unknown_message_type_does_not_panic() {
        let text = r#"[{"method":"properties","params":{"name":"/foo","ack":true}}]"#;
        let msgs = parse_server_messages(text);
        assert_eq!(msgs.len(), 1);
        assert!(matches!(msgs[0], ServerMessage::Other));
    }

    #[test]
    fn binary_frame_roundtrip() {
        // Build a synthetic frame: topic 7, t=12345, type=1 (double), value=3.14
        let mut buf = Vec::new();
        let v = rmpv::Value::Array(vec![
            7u32.into(),
            12345u64.into(),
            1u64.into(),
            rmpv::Value::F64(3.14),
        ]);
        rmpv::encode::write_value(&mut buf, &v).unwrap();

        let updates = parse_binary_frame(&buf);
        assert_eq!(updates.len(), 1);
        assert_eq!(updates[0].topic_id, 7);
        assert_eq!(updates[0].timestamp_us, 12345);
        match updates[0].value {
            Value::Double(d) => assert!((d - 3.14).abs() < 1e-9),
            _ => panic!("wrong type"),
        }
    }
}