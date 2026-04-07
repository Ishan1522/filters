use std::collections::HashMap;

#[derive(Debug, Clone)]
pub enum SampleValue {
    Double(f64),
    Float(f32),
    Int64(i64),
    Boolean(bool),
    StringVal(String),
}

#[derive(Debug, Clone)]
pub struct Sample {
    pub timestamp_us: u64,
    pub value: SampleValue,
}

#[derive(Debug, Clone)]
pub struct Channel {
    pub entry_id: u32,
    pub name: String,
    pub data_type: String,
    pub metadata: String,
}

#[derive(Debug)]
pub struct LogFile {
    pub channels: Vec<Channel>,
    pub data: HashMap<u32, Vec<Sample>>,
}

struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }

    fn read_bytes(&mut self, n: usize) -> Option<&[u8]> {
        if self.remaining() < n { return None; }
        let slice = &self.buf[self.pos..self.pos + n];
        self.pos += n;
        Some(slice)
    }

    fn read_u8(&mut self) -> Option<u8> {
        Some(self.read_bytes(1)?[0])
    }

    fn read_u16_le(&mut self) -> Option<u16> {
        let b = self.read_bytes(2)?;
        Some(u16::from_le_bytes(b.try_into().ok()?))
    }

    fn read_u32_le(&mut self) -> Option<u32> {
        let b = self.read_bytes(4)?;
        Some(u32::from_le_bytes(b.try_into().ok()?))
    }

    fn read_f32(&mut self) -> Option<f32> {
        let b = self.read_bytes(4)?;
        Some(f32::from_le_bytes(b.try_into().ok()?))
    }

    fn read_f64(&mut self) -> Option<f64> {
        let b = self.read_bytes(8)?;
        Some(f64::from_le_bytes(b.try_into().ok()?))
    }

    fn read_i64_le(&mut self) -> Option<i64> {
        let b = self.read_bytes(8)?;
        Some(i64::from_le_bytes(b.try_into().ok()?))
    }

    /// Read a little-endian unsigned integer of `n` bytes (1..=8) into a u64.
    /// WPILOG uses these for entry id, payload size, and timestamp, with the
    /// width specified by the header bitfield byte at the start of each record.
    fn read_uint_le(&mut self, n: usize) -> Option<u64> {
        let bytes = self.read_bytes(n)?;
        let mut out = 0u64;
        for (i, &b) in bytes.iter().enumerate() {
            out |= (b as u64) << (i * 8);
        }
        Some(out)
    }

    fn read_wpi_string(&mut self) -> Option<String> {
        let len = self.read_u32_le()? as usize;
        let bytes = self.read_bytes(len)?;
        String::from_utf8(bytes.to_vec()).ok()
    }
}

pub fn load(path: &str) -> Option<LogFile> {
    let buf = std::fs::read(path).ok()?;
    let mut r = Reader::new(&buf);

    let magic = r.read_bytes(6)?;
    if magic != b"WPILOG" {
        eprintln!("not a wpilog file");
        return None;
    }

    let version = r.read_u16_le()?;
    if version != 0x0100 {
        eprintln!("unsupported version: {:#06x}", version);
        return None;
    }

    let extra_len = r.read_u32_le()? as usize;
    r.read_bytes(extra_len)?;

    let mut log = LogFile {
        channels: Vec::new(),
        data: HashMap::new(),
    };
    let mut id_to_channel: HashMap<u32, usize> = HashMap::new();

    while r.remaining() > 0 {
        // Header bitfield byte: encodes the widths of the next three fields.
        // Bits 0..=1: entry-id length minus 1     → 1..=4 bytes
        // Bits 2..=3: payload-size length minus 1 → 1..=4 bytes
        // Bits 4..=6: timestamp length minus 1    → 1..=8 bytes
        let header_byte = match r.read_u8() {
            Some(b) => b,
            None => break,
        };
        let entry_len     = ((header_byte        & 0x03) as usize) + 1;
        let size_len      = (((header_byte >> 2) & 0x03) as usize) + 1;
        let timestamp_len = (((header_byte >> 4) & 0x07) as usize) + 1;

        let entry_id    = match r.read_uint_le(entry_len)     { Some(v) => v as u32, None => break };
        let payload_len = match r.read_uint_le(size_len)      { Some(v) => v as usize, None => break };
        let timestamp   = match r.read_uint_le(timestamp_len) { Some(v) => v, None => break };

        // Bounds-check the payload before slicing it.
        if r.pos + payload_len > buf.len() {
            println!("payload out of bounds at pos {} (need {})", r.pos, payload_len);
            break;
        }
        let payload_start = r.pos;
        r.pos += payload_len;
        let mut p = Reader::new(&buf[payload_start..payload_start + payload_len]);

        if entry_id == 0 {
            // Control record. First byte is the control type.
            let control_type = match p.read_u8() {
                Some(v) => v,
                None => continue,
            };
            if control_type == 0 {
                // Start record: entry_id (u32), name (wpi string), type (wpi string), metadata (wpi string)
                let id    = match p.read_u32_le()     { Some(v) => v, None => continue };
                let name  = match p.read_wpi_string() { Some(v) => v, None => continue };
                let dtype = match p.read_wpi_string() { Some(v) => v, None => continue };
                let meta  = match p.read_wpi_string() { Some(v) => v, None => continue };

                let idx = log.channels.len();
                id_to_channel.insert(id, idx);
                log.channels.push(Channel {
                    entry_id: id,
                    name,
                    data_type: dtype,
                    metadata: meta,
                });
            }
            // control_type == 1 is "finish record", == 2 is "set metadata".
            // We don't need to handle them for visualization.
            continue;
        }

        // Data record. Look up the channel and decode based on its type.
        let ch_idx = match id_to_channel.get(&entry_id) {
            Some(&i) => i,
            None => continue,
        };
        let dtype = log.channels[ch_idx].data_type.clone();

        let value = match dtype.as_str() {
            "double"  => match p.read_f64()        { Some(v) => SampleValue::Double(v),       None => continue },
            "float"   => match p.read_f32()        { Some(v) => SampleValue::Float(v),        None => continue },
            "int64"   => match p.read_i64_le()     { Some(v) => SampleValue::Int64(v),        None => continue },
            "boolean" => match p.read_u8()         { Some(v) => SampleValue::Boolean(v != 0), None => continue },
            "string"  => match p.read_wpi_string() { Some(v) => SampleValue::StringVal(v),    None => continue },
            _ => continue,
        };

        log.data
            .entry(entry_id)
            .or_insert_with(Vec::new)
            .push(Sample { timestamp_us: timestamp, value });
    }

    println!("parsed {} channels", log.channels.len());
    Some(log)
}