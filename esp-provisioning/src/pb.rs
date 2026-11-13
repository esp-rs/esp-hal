//! Shared protobuf varint helpers for protocomm and sec1.

/// Read an unsigned varint from `data` at `*i`, advancing `*i`.
pub(crate) fn read_varint(data: &[u8], i: &mut usize) -> Option<u64> {
    let mut v: u64 = 0;
    let mut shift = 0u32;
    while *i < data.len() {
        let b = data[*i];
        *i += 1;
        v |= u64::from(b & 0x7F) << shift;
        if b & 0x80 == 0 {
            return Some(v);
        }
        shift += 7;
        if shift >= 64 {
            return None;
        }
    }
    None
}

/// Write a varint into `buf`, returning the number of bytes written.
pub(crate) fn write_varint(mut val: u64, buf: &mut [u8]) -> usize {
    let mut pos = 0;
    loop {
        if pos >= buf.len() {
            break;
        }
        let mut byte = (val & 0x7F) as u8;
        val >>= 7;
        if val != 0 {
            byte |= 0x80;
        }
        buf[pos] = byte;
        pos += 1;
        if val == 0 {
            break;
        }
    }
    pos
}
