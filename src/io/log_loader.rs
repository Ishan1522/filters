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

    fn read_varint(&mut self) -> Option<u64> {
        let mut result = 0u64;
        let mut shift = 0;
        loop {
            let byte = *self.read_bytes(1)?.first()?;
            result |= ((byte & 0x7F) as u64) << shift;
            if byte & 0x80 == 0 { break; }
            shift += 7;
        }
        Some(result)
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
        let header_len = match r.read_varint() {
            Some(v) => v as usize,
            None => { println!("failed reading header_len at pos {}", r.pos); break; }
        };
        let payload_len = match r.read_varint() {
            Some(v) => v as usize,
            None => { println!("failed reading payload_len at pos {}", r.pos); break; }
        };

        let header_start = r.pos;

        let entry_id = match r.read_varint() {
            Some(v) => v as u32,
            None => { println!("failed reading entry_id at pos {}", r.pos); break; }
        };
        let timestamp = match r.read_varint() {
            Some(v) => v,
            None => { println!("failed reading timestamp at pos {}", r.pos); break; }
        };

        // skip remaining header bytes (forward compat)
        r.pos = header_start + header_len;

        let payload_start = r.pos;
        r.pos += payload_len;

        if r.pos > buf.len() {
            println!("record payload out of bounds at pos {}", r.pos);
            break;
        }

        let mut p = Reader::new(&buf[payload_start..payload_start + payload_len]);

       if entry_id == 0 {
    let control_type = match p.read_u32_le() {  // was read_u8
    Some(v) => v,
    None => { println!("failed reading control_type"); continue; }
};

if control_type == 0 {
    let id = match p.read_u32_le() {  // this stays
        Some(v) => v,
                    None => { println!("failed reading channel id"); continue; }
                };
                let name = match p.read_wpi_string() {
                    Some(v) => v,
                    None => { println!("failed reading channel name"); continue; }
                };
                let dtype = match p.read_wpi_string() {
                    Some(v) => v,
                    None => { println!("failed reading channel type"); continue; }
                };
                let meta = match p.read_wpi_string() {
                    Some(v) => v,
                    None => { println!("failed reading channel meta"); continue; }
                };

                let idx = log.channels.len();
                id_to_channel.insert(id, idx);
                log.channels.push(Channel {
                    entry_id: id,
                    name,
                    data_type: dtype,
                    metadata: meta,
                });
            }
            continue;
        }

        let ch_idx = match id_to_channel.get(&entry_id) {
            Some(&i) => i,
            None => continue,
        };

        let dtype = log.channels[ch_idx].data_type.clone();

        let value = match dtype.as_str() {
            "double"  => match p.read_f64() {
                Some(v) => SampleValue::Double(v),
                None => continue,
            },
            "float"   => match p.read_f32() {
                Some(v) => SampleValue::Float(v),
                None => continue,
            },
            "int64"   => match p.read_i64_le() {
                Some(v) => SampleValue::Int64(v),
                None => continue,
            },
            "boolean" => match p.read_u8() {
                Some(v) => SampleValue::Boolean(v != 0),
                None => continue,
            },
            "string"  => match p.read_wpi_string() {
                Some(v) => SampleValue::StringVal(v),
                None => continue,
            },
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