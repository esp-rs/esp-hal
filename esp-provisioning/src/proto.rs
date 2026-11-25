//! Protobuf messages for ESP-IDF Wi-Fi provisioning.
//!
//! Sources: vendored from ESP-IDF v5.4.2 `components/wifi_provisioning/proto`
//! and `components/protocomm/proto` (not `network_provisioning`). The generated
//! module is committed so firmware builds do not need `protoc`.

use micropb::{MessageDecode, MessageEncode};

pub(crate) use crate::proto_gen::*;

pub(crate) fn encode(msg: &impl MessageEncode, out: &mut [u8]) -> Result<usize, ()> {
    let full = out.len();
    let written = {
        let mut rest = &mut out[..];
        let mut encoder = micropb::PbEncoder::new(&mut rest);
        msg.encode(&mut encoder)?;
        full - rest.len()
    };
    Ok(written)
}

pub(crate) fn decode<T>(bytes: &[u8]) -> Result<T, ()>
where
    T: MessageDecode + Default,
{
    let mut msg = T::default();
    msg.decode_from_bytes(bytes).map_err(|_| ())?;
    Ok(msg)
}
