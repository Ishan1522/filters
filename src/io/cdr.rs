//! Minimal CDR (Common Data Representation) reader for ROS 2 message payloads.
//!
//! ROS 2 serializes messages with CDR (XCDR1), and rosbag2/MCAP files store
//! exactly those payloads under `message_encoding = "cdr"`. This module
//! implements the subset of CDR needed to decode the message types this app
//! can analyze (scalars, multi-arrays, IMU, odometry, twist, …) — see
//! [`crate::io::rosbag_loader`].
//!
//! Encoding notes (XCDR1, little-endian):
//! - Primitive fields are aligned to their own size **relative to the start of
//!   the message payload** — the same rule FastCDR, CycloneDDS, and
//!   ros2-rust's `cdr` crate all use in practice.
//! - `string` = `uint32 length` + bytes + NUL terminator (the NUL is not
//!   counted in `length`).
//! - `sequence<T>` = `uint32 count` + elements, each element aligned to
//!   `alignof(T)` relative to the message start.
//! - A DDS CDR encapsulation header (`0x00 0x01` = CDR_LE) may precede the
//!   payload; it is skipped automatically when present.

/// Streaming reader over a CDR-encoded byte slice.
pub struct CdrReader<'a> {
    buf: &'a [u8],
    pos: usize,
    /// Absolute offset of the payload start in `buf` (0, or 4 if a DDS CDR
    /// encapsulation header was skipped). CDR alignment is relative to the
    /// payload start, not the raw buffer.
    base: usize,
}

impl<'a> CdrReader<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        let mut r = Self {
            buf,
            pos: 0,
            base: 0,
        };
        // DDS CDR encapsulation header: uint16 representation id (0x0001 =
        // CDR_LE) + uint16 options. Present in DDS wire payloads; rosbag
        // payloads may or may not carry it — detect and skip.
        if r.buf.len() >= 4 && r.buf[0] == 0x00 && r.buf[1] == 0x01 {
            r.pos = 4;
            r.base = 4;
        }
        r
    }

    /// Current byte offset from the start of the payload. Test/diagnostic
    /// helper (used by the unit tests to assert alignment).
    #[cfg(test)]
    pub fn pos(&self) -> usize {
        self.pos
    }

    pub fn remaining(&self) -> usize {
        self.buf.len().saturating_sub(self.pos)
    }

    /// Align the cursor up to a multiple of `alignment` (relative to the
    /// payload start), as CDR requires.
    fn align(&mut self, alignment: usize) -> Option<()> {
        let rem = (self.pos - self.base) % alignment;
        if rem != 0 {
            self.pos += alignment - rem;
        }
        if self.pos > self.buf.len() {
            return None;
        }
        Some(())
    }

    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        if n > self.remaining() {
            return None;
        }
        let slice = &self.buf[self.pos..self.pos + n];
        self.pos += n;
        Some(slice)
    }

    pub fn read_u8(&mut self) -> Option<u8> {
        Some(self.take(1)?[0])
    }

    pub fn read_bool(&mut self) -> Option<bool> {
        Some(self.read_u8()? != 0)
    }

    pub fn read_u32(&mut self) -> Option<u32> {
        self.align(4)?;
        let b: [u8; 4] = self.take(4)?.try_into().ok()?;
        Some(u32::from_le_bytes(b))
    }

    pub fn read_i32(&mut self) -> Option<i32> {
        self.align(4)?;
        let b: [u8; 4] = self.take(4)?.try_into().ok()?;
        Some(i32::from_le_bytes(b))
    }

    pub fn read_i64(&mut self) -> Option<i64> {
        self.align(8)?;
        let b: [u8; 8] = self.take(8)?.try_into().ok()?;
        Some(i64::from_le_bytes(b))
    }

    pub fn read_f32(&mut self) -> Option<f32> {
        self.align(4)?;
        let b: [u8; 4] = self.take(4)?.try_into().ok()?;
        Some(f32::from_le_bytes(b))
    }

    pub fn read_f64(&mut self) -> Option<f64> {
        self.align(8)?;
        let b: [u8; 8] = self.take(8)?.try_into().ok()?;
        Some(f64::from_le_bytes(b))
    }

    /// Read a CDR `string`: uint32 length, bytes, NUL terminator.
    pub fn read_string(&mut self) -> Option<String> {
        let len = self.read_u32()? as usize;
        let bytes = self.take(len)?;
        let s = std::str::from_utf8(bytes).ok()?.to_string();
        // Skip the trailing NUL terminator (not counted in `len`).
        if self.pos < self.buf.len() {
            self.pos += 1;
        }
        Some(s)
    }

    /// Read a `sequence<float64>`: uint32 count, then count doubles.
    pub fn read_f64_seq(&mut self, out: &mut Vec<f64>) -> Option<()> {
        let n = self.read_u32()? as usize;
        for _ in 0..n {
            out.push(self.read_f64()?);
        }
        Some(())
    }

    /// Read a `sequence<string>`: uint32 count, then count strings.
    pub fn read_string_seq(&mut self, out: &mut Vec<String>) -> Option<()> {
        let n = self.read_u32()? as usize;
        for _ in 0..n {
            out.push(self.read_string()?);
        }
        Some(())
    }

    /// Skip `n` bytes (used to step over unused covariance / fixed arrays).
    pub fn skip(&mut self, n: usize) -> Option<()> {
        self.take(n).map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    pub(crate) use crate::io::cdr::test_helpers::CdrWriter;

    #[test]
    fn float64_payload() {
        // std_msgs/msg/Float64: single little-endian double.
        let mut w = CdrWriter::new();
        w.f64(3.14159);
        let mut r = CdrReader::new(&w.buf);
        assert_eq!(r.read_f64(), Some(3.14159));
        assert_eq!(r.remaining(), 0);
    }

    #[test]
    fn encapsulation_header_is_skipped() {
        let mut w = CdrWriter::new();
        w.f64(1.0);
        // Prepend a DDS CDR_LE encapsulation header.
        let mut payload = vec![0x00, 0x01, 0x00, 0x00];
        payload.extend_from_slice(&w.buf);
        let mut r = CdrReader::new(&payload);
        assert_eq!(r.read_f64(), Some(1.0));
    }

    #[test]
    fn header_plus_vector3_matches_ros_layout() {
        // Mimic a message that starts with std_msgs/Header (int32 sec,
        // uint32 nanosec, string frame_id) followed by a Vector3 (3 f64).
        // This is the exact layout of e.g. geometry_msgs/PointStamped.
        let mut w = CdrWriter::new();
        w.u32(1234); // header.stamp.sec (int32)
        w.u32(5678); // header.stamp.nanosec (uint32)
        w.string("base_link");
        w.f64(1.0); // vector.x
        w.f64(2.0); // vector.y
        w.f64(3.0); // vector.z

        let mut r = CdrReader::new(&w.buf);
        assert_eq!(r.read_i32(), Some(1234));
        assert_eq!(r.read_u32(), Some(5678));
        assert_eq!(r.read_string().as_deref(), Some("base_link"));
        // The string ends at a 4-boundary (offset 22); the reader must pad to
        // an 8-boundary before the Vector3 — the values below are only
        // correct if that alignment happened.
        assert_eq!(r.read_f64(), Some(1.0));
        assert_eq!(r.read_f64(), Some(2.0));
        assert_eq!(r.read_f64(), Some(3.0));
        assert_eq!(r.remaining(), 0);
    }

    #[test]
    fn f64_sequence_elements_are_absolutely_aligned() {
        // [f64 x][u32 count][f64 elements] puts the sequence count at absolute
        // offset 8, so the first element starts at offset 12 (≡ 4 mod 8).
        // Absolute alignment (FastCDR/CycloneDDS behavior) pads 4 bytes and
        // places the first element at offset 16; relative-to-sequence
        // alignment would place it at 12. The reader must follow the former.
        let mut w = CdrWriter::new();
        w.f64(0.0); // some 8-byte field before the sequence
        w.f64_seq(&[1.5, 2.5, 3.5]);

        let mut r = CdrReader::new(&w.buf);
        assert_eq!(r.read_f64(), Some(0.0));
        assert_eq!(r.pos(), 8);
        let mut data = Vec::new();
        r.read_f64_seq(&mut data).unwrap();
        assert_eq!(r.pos() % 8, 0, "first element must be 8-aligned");
        assert_eq!(data, vec![1.5, 2.5, 3.5]);
    }

    #[test]
    fn imu_layout_is_decoded_field_by_field() {
        // sensor_msgs/msg/Imu is a flat run of doubles after the header;
        // decode the quaternion + angular velocity + linear acceleration.
        let mut w = CdrWriter::new();
        w.u32(1); // stamp.sec
        w.u32(2); // stamp.nanosec
        w.string("imu");
        // orientation (Quaternion: x,y,z,w)
        w.f64(0.0);
        w.f64(0.0);
        w.f64(0.0);
        w.f64(1.0);
        // orientation_covariance: float64[9] — skip via 9 doubles
        for _ in 0..9 {
            w.f64(0.0);
        }
        // angular_velocity (Vector3)
        w.f64(0.1);
        w.f64(0.2);
        w.f64(0.3);
        // angular_velocity_covariance[9]
        for _ in 0..9 {
            w.f64(0.0);
        }
        // linear_acceleration (Vector3)
        w.f64(9.81);
        w.f64(0.0);
        w.f64(-0.5);

        let mut r = CdrReader::new(&w.buf);
        assert_eq!(r.read_i32(), Some(1));
        assert_eq!(r.read_u32(), Some(2));
        assert_eq!(r.read_string().as_deref(), Some("imu"));
        let q = [
            r.read_f64().unwrap(),
            r.read_f64().unwrap(),
            r.read_f64().unwrap(),
            r.read_f64().unwrap(),
        ];
        assert_eq!(q, [0.0, 0.0, 0.0, 1.0]);
        r.skip(9 * 8).unwrap();
        assert_eq!(r.read_f64(), Some(0.1));
        assert_eq!(r.read_f64(), Some(0.2));
        assert_eq!(r.read_f64(), Some(0.3));
        r.skip(9 * 8).unwrap();
        assert_eq!(r.read_f64(), Some(9.81));
        assert_eq!(r.read_f64(), Some(0.0));
        assert_eq!(r.read_f64(), Some(-0.5));
        assert_eq!(r.remaining(), 0);
    }

    #[test]
    fn truncated_payload_returns_none() {
        // read_f64 needs 8 bytes but only 5 remain.
        let mut r = CdrReader::new(&[0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(r.read_f64(), None);
        // A string length prefix claiming more bytes than remain.
        let mut r = CdrReader::new(&[0x05, 0x00, 0x00, 0x00, b'x']);
        assert_eq!(r.read_string(), None);
    }
}

/// Hand-rolled CDR encoder mirroring the wire format [`CdrReader`] expects
/// (absolute alignment, NUL-terminated strings). Shared with the rosbag
/// loader tests to build realistic payloads; also documents the exact byte
/// layout of each message type.
#[cfg(test)]
pub(crate) mod test_helpers {
    pub struct CdrWriter {
        pub buf: Vec<u8>,
    }

    impl CdrWriter {
        pub fn new() -> Self {
            Self { buf: Vec::new() }
        }
        fn align(&mut self, alignment: usize) {
            while self.buf.len() % alignment != 0 {
                self.buf.push(0);
            }
        }
        pub fn u32(&mut self, v: u32) {
            self.align(4);
            self.buf.extend_from_slice(&v.to_le_bytes());
        }
        pub fn f64(&mut self, v: f64) {
            self.align(8);
            self.buf.extend_from_slice(&v.to_le_bytes());
        }
        pub fn string(&mut self, s: &str) {
            self.u32(s.len() as u32);
            self.buf.extend_from_slice(s.as_bytes());
            self.buf.push(0);
        }
        pub fn f64_seq(&mut self, vals: &[f64]) {
            self.u32(vals.len() as u32);
            for v in vals {
                self.f64(*v);
            }
        }
    }
}
