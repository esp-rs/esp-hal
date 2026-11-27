/// Allowed values for the status
/// of a protocomm instance
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct Status(pub i32);
impl Status {
    /// Maximum encoded size of the enum
    pub const _MAX_SIZE: usize = 10usize;
    pub const Success: Self = Self(0);
    pub const InvalidSecScheme: Self = Self(1);
    pub const InvalidProto: Self = Self(2);
    pub const TooManySessions: Self = Self(3);
    pub const InvalidArgument: Self = Self(4);
    pub const InternalError: Self = Self(5);
    pub const CryptoError: Self = Self(6);
    pub const InvalidSession: Self = Self(7);
}
impl core::default::Default for Status {
    fn default() -> Self {
        Self(0)
    }
}
impl core::convert::From<i32> for Status {
    fn from(val: i32) -> Self {
        Self(val)
    }
}
impl core::fmt::Debug for Status {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            Status::Success => formatter.write_str("Success"),
            Status::InvalidSecScheme => formatter.write_str("InvalidSecScheme"),
            Status::InvalidProto => formatter.write_str("InvalidProto"),
            Status::TooManySessions => formatter.write_str("TooManySessions"),
            Status::InvalidArgument => formatter.write_str("InvalidArgument"),
            Status::InternalError => formatter.write_str("InternalError"),
            Status::CryptoError => formatter.write_str("CryptoError"),
            Status::InvalidSession => formatter.write_str("InvalidSession"),
            Self(n) => formatter.debug_tuple("_Unknown").field(&n).finish(),
        }
    }
}
/// Data structure of Session command/request packet
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct S0SessionCmd {}
impl S0SessionCmd {}
impl ::micropb::MessageDecode for S0SessionCmd {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for S0SessionCmd {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        size
    }
}
/// Data structure of Session response packet
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct S0SessionResp {
    pub r#status: Status,
}
impl S0SessionResp {
    /// Return a reference to `status`
    #[inline]
    pub fn r#status(&self) -> &Status {
        &self.r#status
    }
    /// Return a mutable reference to `status`
    #[inline]
    pub fn mut_status(&mut self) -> &mut Status {
        &mut self.r#status
    }
    /// Set the value of `status`
    #[inline]
    pub fn set_status(&mut self, value: Status) -> &mut Self {
        self.r#status = value.into();
        self
    }
    /// Builder method that sets the value of `status`. Useful for initializing the message.
    #[inline]
    pub fn init_status(mut self, value: Status) -> Self {
        self.r#status = value.into();
        self
    }
}
impl ::micropb::MessageDecode for S0SessionResp {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#status;
                    {
                        let val = decoder.decode_int32().map(|n| Status(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for S0SessionResp {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(Status::_MAX_SIZE), |size| size
            + 1usize)
        {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        size
    }
}
/// Payload structure of session data
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct Sec0Payload {
    /// !< Type of message
    pub r#msg: Sec0MsgType,
    pub r#payload: ::core::option::Option<Sec0Payload_::Payload>,
}
impl Sec0Payload {
    /// Return a reference to `msg`
    #[inline]
    pub fn r#msg(&self) -> &Sec0MsgType {
        &self.r#msg
    }
    /// Return a mutable reference to `msg`
    #[inline]
    pub fn mut_msg(&mut self) -> &mut Sec0MsgType {
        &mut self.r#msg
    }
    /// Set the value of `msg`
    #[inline]
    pub fn set_msg(&mut self, value: Sec0MsgType) -> &mut Self {
        self.r#msg = value.into();
        self
    }
    /// Builder method that sets the value of `msg`. Useful for initializing the message.
    #[inline]
    pub fn init_msg(mut self, value: Sec0MsgType) -> Self {
        self.r#msg = value.into();
        self
    }
}
impl ::micropb::MessageDecode for Sec0Payload {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#msg;
                    {
                        let val = decoder.decode_int32().map(|n| Sec0MsgType(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                20u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let Sec0Payload_::Payload::Sc(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(Sec0Payload_::Payload::Sc(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                21u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let Sec0Payload_::Payload::Sr(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(Sec0Payload_::Payload::Sr(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for Sec0Payload {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(
            ::core::result::Result::Ok(Sec0MsgType::_MAX_SIZE),
            |size| size + 1usize
        ) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match 'oneof: {
            let mut max_size = 0;
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <S0SessionCmd as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 2usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <S0SessionResp as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 2usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            ::core::result::Result::Ok(max_size)
        } {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#msg;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        if let Some(oneof) = &self.r#payload {
            match &*oneof {
                Sec0Payload_::Payload::Sc(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(162u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                Sec0Payload_::Payload::Sr(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(170u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#msg;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        if let Some(oneof) = &self.r#payload {
            match &*oneof {
                Sec0Payload_::Payload::Sc(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 2usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                Sec0Payload_::Payload::Sr(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 2usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
            }
        }
        size
    }
}
/// Inner types for `Sec0Payload`
pub mod Sec0Payload_ {
    #[derive(Debug, PartialEq, Clone, Copy)]
    pub enum Payload {
        /// !< Payload data interpreted as Cmd
        Sc(super::S0SessionCmd),
        /// !< Payload data interpreted as Resp
        Sr(super::S0SessionResp),
    }
}
/// A message must be of type Cmd or Resp
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct Sec0MsgType(pub i32);
impl Sec0MsgType {
    /// Maximum encoded size of the enum
    pub const _MAX_SIZE: usize = 10usize;
    pub const S0SessionCommand: Self = Self(0);
    pub const S0SessionResponse: Self = Self(1);
}
impl core::default::Default for Sec0MsgType {
    fn default() -> Self {
        Self(0)
    }
}
impl core::convert::From<i32> for Sec0MsgType {
    fn from(val: i32) -> Self {
        Self(val)
    }
}
impl core::fmt::Debug for Sec0MsgType {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            Sec0MsgType::S0SessionCommand => formatter.write_str("S0SessionCommand"),
            Sec0MsgType::S0SessionResponse => formatter.write_str("S0SessionResponse"),
            Self(n) => formatter.debug_tuple("_Unknown").field(&n).finish(),
        }
    }
}
/// Data structure of Session command1 packet
#[derive(Debug, Default, PartialEq, Clone)]
pub struct SessionCmd1 {
    pub r#client_verify_data: ::heapless::Vec<u8, 64>,
}
impl SessionCmd1 {
    /// Return a reference to `client_verify_data`
    #[inline]
    pub fn r#client_verify_data(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#client_verify_data
    }
    /// Return a mutable reference to `client_verify_data`
    #[inline]
    pub fn mut_client_verify_data(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#client_verify_data
    }
    /// Set the value of `client_verify_data`
    #[inline]
    pub fn set_client_verify_data(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#client_verify_data = value.into();
        self
    }
    /// Builder method that sets the value of `client_verify_data`. Useful for initializing the
    /// message.
    #[inline]
    pub fn init_client_verify_data(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#client_verify_data = value.into();
        self
    }
}
impl ::micropb::MessageDecode for SessionCmd1 {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                2u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#client_verify_data;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for SessionCmd1 {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#client_verify_data;
            if !val_ref.is_empty() {
                encoder.encode_varint32(18u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#client_verify_data;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        size
    }
}
/// Data structure of Session response1 packet
#[derive(Debug, Default, PartialEq, Clone)]
pub struct SessionResp1 {
    pub r#status: Status,
    pub r#device_verify_data: ::heapless::Vec<u8, 64>,
}
impl SessionResp1 {
    /// Return a reference to `status`
    #[inline]
    pub fn r#status(&self) -> &Status {
        &self.r#status
    }
    /// Return a mutable reference to `status`
    #[inline]
    pub fn mut_status(&mut self) -> &mut Status {
        &mut self.r#status
    }
    /// Set the value of `status`
    #[inline]
    pub fn set_status(&mut self, value: Status) -> &mut Self {
        self.r#status = value.into();
        self
    }
    /// Builder method that sets the value of `status`. Useful for initializing the message.
    #[inline]
    pub fn init_status(mut self, value: Status) -> Self {
        self.r#status = value.into();
        self
    }
    /// Return a reference to `device_verify_data`
    #[inline]
    pub fn r#device_verify_data(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#device_verify_data
    }
    /// Return a mutable reference to `device_verify_data`
    #[inline]
    pub fn mut_device_verify_data(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#device_verify_data
    }
    /// Set the value of `device_verify_data`
    #[inline]
    pub fn set_device_verify_data(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#device_verify_data = value.into();
        self
    }
    /// Builder method that sets the value of `device_verify_data`. Useful for initializing the
    /// message.
    #[inline]
    pub fn init_device_verify_data(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#device_verify_data = value.into();
        self
    }
}
impl ::micropb::MessageDecode for SessionResp1 {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#status;
                    {
                        let val = decoder.decode_int32().map(|n| Status(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                3u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#device_verify_data;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for SessionResp1 {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(Status::_MAX_SIZE), |size| size
            + 1usize)
        {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        {
            let val_ref = &self.r#device_verify_data;
            if !val_ref.is_empty() {
                encoder.encode_varint32(26u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        {
            let val_ref = &self.r#device_verify_data;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        size
    }
}
/// Data structure of Session command0 packet
#[derive(Debug, Default, PartialEq, Clone)]
pub struct SessionCmd0 {
    pub r#client_pubkey: ::heapless::Vec<u8, 64>,
}
impl SessionCmd0 {
    /// Return a reference to `client_pubkey`
    #[inline]
    pub fn r#client_pubkey(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#client_pubkey
    }
    /// Return a mutable reference to `client_pubkey`
    #[inline]
    pub fn mut_client_pubkey(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#client_pubkey
    }
    /// Set the value of `client_pubkey`
    #[inline]
    pub fn set_client_pubkey(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#client_pubkey = value.into();
        self
    }
    /// Builder method that sets the value of `client_pubkey`. Useful for initializing the message.
    #[inline]
    pub fn init_client_pubkey(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#client_pubkey = value.into();
        self
    }
}
impl ::micropb::MessageDecode for SessionCmd0 {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#client_pubkey;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for SessionCmd0 {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#client_pubkey;
            if !val_ref.is_empty() {
                encoder.encode_varint32(10u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#client_pubkey;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        size
    }
}
/// Data structure of Session response0 packet
#[derive(Debug, Default, PartialEq, Clone)]
pub struct SessionResp0 {
    pub r#status: Status,
    pub r#device_pubkey: ::heapless::Vec<u8, 64>,
    pub r#device_random: ::heapless::Vec<u8, 64>,
}
impl SessionResp0 {
    /// Return a reference to `status`
    #[inline]
    pub fn r#status(&self) -> &Status {
        &self.r#status
    }
    /// Return a mutable reference to `status`
    #[inline]
    pub fn mut_status(&mut self) -> &mut Status {
        &mut self.r#status
    }
    /// Set the value of `status`
    #[inline]
    pub fn set_status(&mut self, value: Status) -> &mut Self {
        self.r#status = value.into();
        self
    }
    /// Builder method that sets the value of `status`. Useful for initializing the message.
    #[inline]
    pub fn init_status(mut self, value: Status) -> Self {
        self.r#status = value.into();
        self
    }
    /// Return a reference to `device_pubkey`
    #[inline]
    pub fn r#device_pubkey(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#device_pubkey
    }
    /// Return a mutable reference to `device_pubkey`
    #[inline]
    pub fn mut_device_pubkey(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#device_pubkey
    }
    /// Set the value of `device_pubkey`
    #[inline]
    pub fn set_device_pubkey(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#device_pubkey = value.into();
        self
    }
    /// Builder method that sets the value of `device_pubkey`. Useful for initializing the message.
    #[inline]
    pub fn init_device_pubkey(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#device_pubkey = value.into();
        self
    }
    /// Return a reference to `device_random`
    #[inline]
    pub fn r#device_random(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#device_random
    }
    /// Return a mutable reference to `device_random`
    #[inline]
    pub fn mut_device_random(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#device_random
    }
    /// Set the value of `device_random`
    #[inline]
    pub fn set_device_random(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#device_random = value.into();
        self
    }
    /// Builder method that sets the value of `device_random`. Useful for initializing the message.
    #[inline]
    pub fn init_device_random(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#device_random = value.into();
        self
    }
}
impl ::micropb::MessageDecode for SessionResp0 {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#status;
                    {
                        let val = decoder.decode_int32().map(|n| Status(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                2u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#device_pubkey;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                3u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#device_random;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for SessionResp0 {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(Status::_MAX_SIZE), |size| size
            + 1usize)
        {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        {
            let val_ref = &self.r#device_pubkey;
            if !val_ref.is_empty() {
                encoder.encode_varint32(18u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#device_random;
            if !val_ref.is_empty() {
                encoder.encode_varint32(26u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        {
            let val_ref = &self.r#device_pubkey;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#device_random;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        size
    }
}
/// Payload structure of session data
#[derive(Debug, Default, PartialEq, Clone)]
pub struct Sec1Payload {
    /// !< Type of message
    pub r#msg: Sec1MsgType,
    pub r#payload: ::core::option::Option<Sec1Payload_::Payload>,
}
impl Sec1Payload {
    /// Return a reference to `msg`
    #[inline]
    pub fn r#msg(&self) -> &Sec1MsgType {
        &self.r#msg
    }
    /// Return a mutable reference to `msg`
    #[inline]
    pub fn mut_msg(&mut self) -> &mut Sec1MsgType {
        &mut self.r#msg
    }
    /// Set the value of `msg`
    #[inline]
    pub fn set_msg(&mut self, value: Sec1MsgType) -> &mut Self {
        self.r#msg = value.into();
        self
    }
    /// Builder method that sets the value of `msg`. Useful for initializing the message.
    #[inline]
    pub fn init_msg(mut self, value: Sec1MsgType) -> Self {
        self.r#msg = value.into();
        self
    }
}
impl ::micropb::MessageDecode for Sec1Payload {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#msg;
                    {
                        let val = decoder.decode_int32().map(|n| Sec1MsgType(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                20u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let Sec1Payload_::Payload::Sc0(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(Sec1Payload_::Payload::Sc0(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                21u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let Sec1Payload_::Payload::Sr0(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(Sec1Payload_::Payload::Sr0(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                22u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let Sec1Payload_::Payload::Sc1(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(Sec1Payload_::Payload::Sc1(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                23u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let Sec1Payload_::Payload::Sr1(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(Sec1Payload_::Payload::Sr1(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for Sec1Payload {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(
            ::core::result::Result::Ok(Sec1MsgType::_MAX_SIZE),
            |size| size + 1usize
        ) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match 'oneof: {
            let mut max_size = 0;
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <SessionCmd0 as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 2usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <SessionResp0 as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 2usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <SessionCmd1 as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 2usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <SessionResp1 as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 2usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            ::core::result::Result::Ok(max_size)
        } {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#msg;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        if let Some(oneof) = &self.r#payload {
            match &*oneof {
                Sec1Payload_::Payload::Sc0(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(162u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                Sec1Payload_::Payload::Sr0(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(170u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                Sec1Payload_::Payload::Sc1(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(178u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                Sec1Payload_::Payload::Sr1(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(186u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#msg;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        if let Some(oneof) = &self.r#payload {
            match &*oneof {
                Sec1Payload_::Payload::Sc0(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 2usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                Sec1Payload_::Payload::Sr0(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 2usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                Sec1Payload_::Payload::Sc1(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 2usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                Sec1Payload_::Payload::Sr1(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 2usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
            }
        }
        size
    }
}
/// Inner types for `Sec1Payload`
pub mod Sec1Payload_ {
    #[derive(Debug, PartialEq, Clone)]
    pub enum Payload {
        /// !< Payload data interpreted as Cmd0
        Sc0(super::SessionCmd0),
        /// !< Payload data interpreted as Resp0
        Sr0(super::SessionResp0),
        /// !< Payload data interpreted as Cmd1
        Sc1(super::SessionCmd1),
        /// !< Payload data interpreted as Resp1
        Sr1(super::SessionResp1),
    }
}
/// A message must be of type Cmd0 / Cmd1 / Resp0 / Resp1
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct Sec1MsgType(pub i32);
impl Sec1MsgType {
    /// Maximum encoded size of the enum
    pub const _MAX_SIZE: usize = 10usize;
    pub const SessionCommand0: Self = Self(0);
    pub const SessionResponse0: Self = Self(1);
    pub const SessionCommand1: Self = Self(2);
    pub const SessionResponse1: Self = Self(3);
}
impl core::default::Default for Sec1MsgType {
    fn default() -> Self {
        Self(0)
    }
}
impl core::convert::From<i32> for Sec1MsgType {
    fn from(val: i32) -> Self {
        Self(val)
    }
}
impl core::fmt::Debug for Sec1MsgType {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            Sec1MsgType::SessionCommand0 => formatter.write_str("SessionCommand0"),
            Sec1MsgType::SessionResponse0 => formatter.write_str("SessionResponse0"),
            Sec1MsgType::SessionCommand1 => formatter.write_str("SessionCommand1"),
            Sec1MsgType::SessionResponse1 => formatter.write_str("SessionResponse1"),
            Self(n) => formatter.debug_tuple("_Unknown").field(&n).finish(),
        }
    }
}
/// Data structure of Session command0 packet
#[derive(Debug, Default, PartialEq, Clone)]
pub struct S2SessionCmd0 {
    pub r#client_username: ::heapless::Vec<u8, 64>,
    pub r#client_pubkey: ::heapless::Vec<u8, 64>,
}
impl S2SessionCmd0 {
    /// Return a reference to `client_username`
    #[inline]
    pub fn r#client_username(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#client_username
    }
    /// Return a mutable reference to `client_username`
    #[inline]
    pub fn mut_client_username(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#client_username
    }
    /// Set the value of `client_username`
    #[inline]
    pub fn set_client_username(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#client_username = value.into();
        self
    }
    /// Builder method that sets the value of `client_username`. Useful for initializing the
    /// message.
    #[inline]
    pub fn init_client_username(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#client_username = value.into();
        self
    }
    /// Return a reference to `client_pubkey`
    #[inline]
    pub fn r#client_pubkey(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#client_pubkey
    }
    /// Return a mutable reference to `client_pubkey`
    #[inline]
    pub fn mut_client_pubkey(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#client_pubkey
    }
    /// Set the value of `client_pubkey`
    #[inline]
    pub fn set_client_pubkey(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#client_pubkey = value.into();
        self
    }
    /// Builder method that sets the value of `client_pubkey`. Useful for initializing the message.
    #[inline]
    pub fn init_client_pubkey(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#client_pubkey = value.into();
        self
    }
}
impl ::micropb::MessageDecode for S2SessionCmd0 {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#client_username;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                2u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#client_pubkey;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for S2SessionCmd0 {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#client_username;
            if !val_ref.is_empty() {
                encoder.encode_varint32(10u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#client_pubkey;
            if !val_ref.is_empty() {
                encoder.encode_varint32(18u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#client_username;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#client_pubkey;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        size
    }
}
/// Data structure of Session response0 packet
#[derive(Debug, Default, PartialEq, Clone)]
pub struct S2SessionResp0 {
    pub r#status: Status,
    pub r#device_pubkey: ::heapless::Vec<u8, 64>,
    pub r#device_salt: ::heapless::Vec<u8, 64>,
}
impl S2SessionResp0 {
    /// Return a reference to `status`
    #[inline]
    pub fn r#status(&self) -> &Status {
        &self.r#status
    }
    /// Return a mutable reference to `status`
    #[inline]
    pub fn mut_status(&mut self) -> &mut Status {
        &mut self.r#status
    }
    /// Set the value of `status`
    #[inline]
    pub fn set_status(&mut self, value: Status) -> &mut Self {
        self.r#status = value.into();
        self
    }
    /// Builder method that sets the value of `status`. Useful for initializing the message.
    #[inline]
    pub fn init_status(mut self, value: Status) -> Self {
        self.r#status = value.into();
        self
    }
    /// Return a reference to `device_pubkey`
    #[inline]
    pub fn r#device_pubkey(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#device_pubkey
    }
    /// Return a mutable reference to `device_pubkey`
    #[inline]
    pub fn mut_device_pubkey(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#device_pubkey
    }
    /// Set the value of `device_pubkey`
    #[inline]
    pub fn set_device_pubkey(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#device_pubkey = value.into();
        self
    }
    /// Builder method that sets the value of `device_pubkey`. Useful for initializing the message.
    #[inline]
    pub fn init_device_pubkey(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#device_pubkey = value.into();
        self
    }
    /// Return a reference to `device_salt`
    #[inline]
    pub fn r#device_salt(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#device_salt
    }
    /// Return a mutable reference to `device_salt`
    #[inline]
    pub fn mut_device_salt(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#device_salt
    }
    /// Set the value of `device_salt`
    #[inline]
    pub fn set_device_salt(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#device_salt = value.into();
        self
    }
    /// Builder method that sets the value of `device_salt`. Useful for initializing the message.
    #[inline]
    pub fn init_device_salt(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#device_salt = value.into();
        self
    }
}
impl ::micropb::MessageDecode for S2SessionResp0 {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#status;
                    {
                        let val = decoder.decode_int32().map(|n| Status(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                2u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#device_pubkey;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                3u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#device_salt;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for S2SessionResp0 {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(Status::_MAX_SIZE), |size| size
            + 1usize)
        {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        {
            let val_ref = &self.r#device_pubkey;
            if !val_ref.is_empty() {
                encoder.encode_varint32(18u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#device_salt;
            if !val_ref.is_empty() {
                encoder.encode_varint32(26u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        {
            let val_ref = &self.r#device_pubkey;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#device_salt;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        size
    }
}
/// Data structure of Session command1 packet
#[derive(Debug, Default, PartialEq, Clone)]
pub struct S2SessionCmd1 {
    pub r#client_proof: ::heapless::Vec<u8, 64>,
}
impl S2SessionCmd1 {
    /// Return a reference to `client_proof`
    #[inline]
    pub fn r#client_proof(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#client_proof
    }
    /// Return a mutable reference to `client_proof`
    #[inline]
    pub fn mut_client_proof(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#client_proof
    }
    /// Set the value of `client_proof`
    #[inline]
    pub fn set_client_proof(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#client_proof = value.into();
        self
    }
    /// Builder method that sets the value of `client_proof`. Useful for initializing the message.
    #[inline]
    pub fn init_client_proof(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#client_proof = value.into();
        self
    }
}
impl ::micropb::MessageDecode for S2SessionCmd1 {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#client_proof;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for S2SessionCmd1 {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#client_proof;
            if !val_ref.is_empty() {
                encoder.encode_varint32(10u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#client_proof;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        size
    }
}
/// Data structure of Session response1 packet
#[derive(Debug, Default, PartialEq, Clone)]
pub struct S2SessionResp1 {
    pub r#status: Status,
    pub r#device_proof: ::heapless::Vec<u8, 64>,
    pub r#device_nonce: ::heapless::Vec<u8, 64>,
}
impl S2SessionResp1 {
    /// Return a reference to `status`
    #[inline]
    pub fn r#status(&self) -> &Status {
        &self.r#status
    }
    /// Return a mutable reference to `status`
    #[inline]
    pub fn mut_status(&mut self) -> &mut Status {
        &mut self.r#status
    }
    /// Set the value of `status`
    #[inline]
    pub fn set_status(&mut self, value: Status) -> &mut Self {
        self.r#status = value.into();
        self
    }
    /// Builder method that sets the value of `status`. Useful for initializing the message.
    #[inline]
    pub fn init_status(mut self, value: Status) -> Self {
        self.r#status = value.into();
        self
    }
    /// Return a reference to `device_proof`
    #[inline]
    pub fn r#device_proof(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#device_proof
    }
    /// Return a mutable reference to `device_proof`
    #[inline]
    pub fn mut_device_proof(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#device_proof
    }
    /// Set the value of `device_proof`
    #[inline]
    pub fn set_device_proof(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#device_proof = value.into();
        self
    }
    /// Builder method that sets the value of `device_proof`. Useful for initializing the message.
    #[inline]
    pub fn init_device_proof(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#device_proof = value.into();
        self
    }
    /// Return a reference to `device_nonce`
    #[inline]
    pub fn r#device_nonce(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#device_nonce
    }
    /// Return a mutable reference to `device_nonce`
    #[inline]
    pub fn mut_device_nonce(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#device_nonce
    }
    /// Set the value of `device_nonce`
    #[inline]
    pub fn set_device_nonce(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#device_nonce = value.into();
        self
    }
    /// Builder method that sets the value of `device_nonce`. Useful for initializing the message.
    #[inline]
    pub fn init_device_nonce(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#device_nonce = value.into();
        self
    }
}
impl ::micropb::MessageDecode for S2SessionResp1 {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#status;
                    {
                        let val = decoder.decode_int32().map(|n| Status(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                2u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#device_proof;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                3u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#device_nonce;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for S2SessionResp1 {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(Status::_MAX_SIZE), |size| size
            + 1usize)
        {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        {
            let val_ref = &self.r#device_proof;
            if !val_ref.is_empty() {
                encoder.encode_varint32(18u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#device_nonce;
            if !val_ref.is_empty() {
                encoder.encode_varint32(26u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        {
            let val_ref = &self.r#device_proof;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#device_nonce;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        size
    }
}
/// Payload structure of session data
#[derive(Debug, Default, PartialEq, Clone)]
pub struct Sec2Payload {
    /// !< Type of message
    pub r#msg: Sec2MsgType,
    pub r#payload: ::core::option::Option<Sec2Payload_::Payload>,
}
impl Sec2Payload {
    /// Return a reference to `msg`
    #[inline]
    pub fn r#msg(&self) -> &Sec2MsgType {
        &self.r#msg
    }
    /// Return a mutable reference to `msg`
    #[inline]
    pub fn mut_msg(&mut self) -> &mut Sec2MsgType {
        &mut self.r#msg
    }
    /// Set the value of `msg`
    #[inline]
    pub fn set_msg(&mut self, value: Sec2MsgType) -> &mut Self {
        self.r#msg = value.into();
        self
    }
    /// Builder method that sets the value of `msg`. Useful for initializing the message.
    #[inline]
    pub fn init_msg(mut self, value: Sec2MsgType) -> Self {
        self.r#msg = value.into();
        self
    }
}
impl ::micropb::MessageDecode for Sec2Payload {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#msg;
                    {
                        let val = decoder.decode_int32().map(|n| Sec2MsgType(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                20u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let Sec2Payload_::Payload::Sc0(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(Sec2Payload_::Payload::Sc0(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                21u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let Sec2Payload_::Payload::Sr0(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(Sec2Payload_::Payload::Sr0(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                22u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let Sec2Payload_::Payload::Sc1(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(Sec2Payload_::Payload::Sc1(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                23u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let Sec2Payload_::Payload::Sr1(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(Sec2Payload_::Payload::Sr1(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for Sec2Payload {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(
            ::core::result::Result::Ok(Sec2MsgType::_MAX_SIZE),
            |size| size + 1usize
        ) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match 'oneof: {
            let mut max_size = 0;
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <S2SessionCmd0 as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 2usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <S2SessionResp0 as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 2usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <S2SessionCmd1 as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 2usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <S2SessionResp1 as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 2usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            ::core::result::Result::Ok(max_size)
        } {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#msg;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        if let Some(oneof) = &self.r#payload {
            match &*oneof {
                Sec2Payload_::Payload::Sc0(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(162u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                Sec2Payload_::Payload::Sr0(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(170u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                Sec2Payload_::Payload::Sc1(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(178u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                Sec2Payload_::Payload::Sr1(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(186u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#msg;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        if let Some(oneof) = &self.r#payload {
            match &*oneof {
                Sec2Payload_::Payload::Sc0(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 2usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                Sec2Payload_::Payload::Sr0(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 2usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                Sec2Payload_::Payload::Sc1(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 2usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                Sec2Payload_::Payload::Sr1(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 2usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
            }
        }
        size
    }
}
/// Inner types for `Sec2Payload`
pub mod Sec2Payload_ {
    #[derive(Debug, PartialEq, Clone)]
    pub enum Payload {
        /// !< Payload data interpreted as Cmd0
        Sc0(super::S2SessionCmd0),
        /// !< Payload data interpreted as Resp0
        Sr0(super::S2SessionResp0),
        /// !< Payload data interpreted as Cmd1
        Sc1(super::S2SessionCmd1),
        /// !< Payload data interpreted as Resp1
        Sr1(super::S2SessionResp1),
    }
}
/// A message must be of type Cmd0 / Cmd1 / Resp0 / Resp1
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct Sec2MsgType(pub i32);
impl Sec2MsgType {
    /// Maximum encoded size of the enum
    pub const _MAX_SIZE: usize = 10usize;
    pub const S2SessionCommand0: Self = Self(0);
    pub const S2SessionResponse0: Self = Self(1);
    pub const S2SessionCommand1: Self = Self(2);
    pub const S2SessionResponse1: Self = Self(3);
}
impl core::default::Default for Sec2MsgType {
    fn default() -> Self {
        Self(0)
    }
}
impl core::convert::From<i32> for Sec2MsgType {
    fn from(val: i32) -> Self {
        Self(val)
    }
}
impl core::fmt::Debug for Sec2MsgType {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            Sec2MsgType::S2SessionCommand0 => formatter.write_str("S2SessionCommand0"),
            Sec2MsgType::S2SessionResponse0 => formatter.write_str("S2SessionResponse0"),
            Sec2MsgType::S2SessionCommand1 => formatter.write_str("S2SessionCommand1"),
            Sec2MsgType::S2SessionResponse1 => formatter.write_str("S2SessionResponse1"),
            Self(n) => formatter.debug_tuple("_Unknown").field(&n).finish(),
        }
    }
}
/// Data structure exchanged when establishing
/// secure session between Host and Client
#[derive(Debug, Default, PartialEq, Clone)]
pub struct SessionData {
    /// !< Type of security
    pub r#sec_ver: SecSchemeVersion,
    pub r#proto: ::core::option::Option<SessionData_::Proto>,
}
impl SessionData {
    /// Return a reference to `sec_ver`
    #[inline]
    pub fn r#sec_ver(&self) -> &SecSchemeVersion {
        &self.r#sec_ver
    }
    /// Return a mutable reference to `sec_ver`
    #[inline]
    pub fn mut_sec_ver(&mut self) -> &mut SecSchemeVersion {
        &mut self.r#sec_ver
    }
    /// Set the value of `sec_ver`
    #[inline]
    pub fn set_sec_ver(&mut self, value: SecSchemeVersion) -> &mut Self {
        self.r#sec_ver = value.into();
        self
    }
    /// Builder method that sets the value of `sec_ver`. Useful for initializing the message.
    #[inline]
    pub fn init_sec_ver(mut self, value: SecSchemeVersion) -> Self {
        self.r#sec_ver = value.into();
        self
    }
}
impl ::micropb::MessageDecode for SessionData {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                2u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#sec_ver;
                    {
                        let val = decoder.decode_int32().map(|n| SecSchemeVersion(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                10u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#proto {
                            if let SessionData_::Proto::Sec0(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#proto = ::core::option::Option::Some(SessionData_::Proto::Sec0(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                11u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#proto {
                            if let SessionData_::Proto::Sec1(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#proto = ::core::option::Option::Some(SessionData_::Proto::Sec1(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                12u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#proto {
                            if let SessionData_::Proto::Sec2(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#proto = ::core::option::Option::Some(SessionData_::Proto::Sec2(
                            ::core::default::Default::default(),
                        ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for SessionData {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(
            ::core::result::Result::Ok(SecSchemeVersion::_MAX_SIZE),
            |size| size + 1usize
        ) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match 'oneof: {
            let mut max_size = 0;
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <Sec0Payload as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <Sec1Payload as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <Sec2Payload as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            ::core::result::Result::Ok(max_size)
        } {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#sec_ver;
            if val_ref.0 != 0 {
                encoder.encode_varint32(16u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        if let Some(oneof) = &self.r#proto {
            match &*oneof {
                SessionData_::Proto::Sec0(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(82u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                SessionData_::Proto::Sec1(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(90u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                SessionData_::Proto::Sec2(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(98u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#sec_ver;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        if let Some(oneof) = &self.r#proto {
            match &*oneof {
                SessionData_::Proto::Sec0(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                SessionData_::Proto::Sec1(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                SessionData_::Proto::Sec2(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
            }
        }
        size
    }
}
/// Inner types for `SessionData`
pub mod SessionData_ {
    #[derive(Debug, PartialEq, Clone)]
    pub enum Proto {
        /// !< Payload data in case of security 0
        Sec0(super::Sec0Payload),
        /// !< Payload data in case of security 1
        Sec1(super::Sec1Payload),
        /// !< Payload data in case of security 2
        Sec2(super::Sec2Payload),
    }
}
/// Allowed values for the type of security
/// being used in a protocomm session
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct SecSchemeVersion(pub i32);
impl SecSchemeVersion {
    /// Maximum encoded size of the enum
    pub const _MAX_SIZE: usize = 10usize;
    /// !< Unsecured - plaintext communication
    pub const SecScheme0: Self = Self(0);
    /// !< Security scheme 1 - Curve25519 + AES-256-CTR
    pub const SecScheme1: Self = Self(1);
    /// !< Security scheme 2 - SRP6a + AES-256-GCM
    pub const SecScheme2: Self = Self(2);
}
impl core::default::Default for SecSchemeVersion {
    fn default() -> Self {
        Self(0)
    }
}
impl core::convert::From<i32> for SecSchemeVersion {
    fn from(val: i32) -> Self {
        Self(val)
    }
}
impl core::fmt::Debug for SecSchemeVersion {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            SecSchemeVersion::SecScheme0 => formatter.write_str("SecScheme0"),
            SecSchemeVersion::SecScheme1 => formatter.write_str("SecScheme1"),
            SecSchemeVersion::SecScheme2 => formatter.write_str("SecScheme2"),
            Self(n) => formatter.debug_tuple("_Unknown").field(&n).finish(),
        }
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct WifiAttemptFailed {
    pub r#attempts_remaining: u32,
}
impl WifiAttemptFailed {
    /// Return a reference to `attempts_remaining`
    #[inline]
    pub fn r#attempts_remaining(&self) -> &u32 {
        &self.r#attempts_remaining
    }
    /// Return a mutable reference to `attempts_remaining`
    #[inline]
    pub fn mut_attempts_remaining(&mut self) -> &mut u32 {
        &mut self.r#attempts_remaining
    }
    /// Set the value of `attempts_remaining`
    #[inline]
    pub fn set_attempts_remaining(&mut self, value: u32) -> &mut Self {
        self.r#attempts_remaining = value.into();
        self
    }
    /// Builder method that sets the value of `attempts_remaining`. Useful for initializing the
    /// message.
    #[inline]
    pub fn init_attempts_remaining(mut self, value: u32) -> Self {
        self.r#attempts_remaining = value.into();
        self
    }
}
impl ::micropb::MessageDecode for WifiAttemptFailed {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#attempts_remaining;
                    {
                        let val = decoder.decode_varint32()?;
                        let val_ref = &val;
                        if *val_ref != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for WifiAttemptFailed {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(5usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#attempts_remaining;
            if *val_ref != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_varint32(*val_ref as _)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#attempts_remaining;
            if *val_ref != 0 {
                size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
            }
        }
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone)]
pub struct WifiConnectedState {
    pub r#ip4_addr: ::heapless::String<64>,
    pub r#auth_mode: WifiAuthMode,
    pub r#ssid: ::heapless::Vec<u8, 64>,
    pub r#bssid: ::heapless::Vec<u8, 64>,
    pub r#channel: i32,
}
impl WifiConnectedState {
    /// Return a reference to `ip4_addr`
    #[inline]
    pub fn r#ip4_addr(&self) -> &::heapless::String<64> {
        &self.r#ip4_addr
    }
    /// Return a mutable reference to `ip4_addr`
    #[inline]
    pub fn mut_ip4_addr(&mut self) -> &mut ::heapless::String<64> {
        &mut self.r#ip4_addr
    }
    /// Set the value of `ip4_addr`
    #[inline]
    pub fn set_ip4_addr(&mut self, value: ::heapless::String<64>) -> &mut Self {
        self.r#ip4_addr = value.into();
        self
    }
    /// Builder method that sets the value of `ip4_addr`. Useful for initializing the message.
    #[inline]
    pub fn init_ip4_addr(mut self, value: ::heapless::String<64>) -> Self {
        self.r#ip4_addr = value.into();
        self
    }
    /// Return a reference to `auth_mode`
    #[inline]
    pub fn r#auth_mode(&self) -> &WifiAuthMode {
        &self.r#auth_mode
    }
    /// Return a mutable reference to `auth_mode`
    #[inline]
    pub fn mut_auth_mode(&mut self) -> &mut WifiAuthMode {
        &mut self.r#auth_mode
    }
    /// Set the value of `auth_mode`
    #[inline]
    pub fn set_auth_mode(&mut self, value: WifiAuthMode) -> &mut Self {
        self.r#auth_mode = value.into();
        self
    }
    /// Builder method that sets the value of `auth_mode`. Useful for initializing the message.
    #[inline]
    pub fn init_auth_mode(mut self, value: WifiAuthMode) -> Self {
        self.r#auth_mode = value.into();
        self
    }
    /// Return a reference to `ssid`
    #[inline]
    pub fn r#ssid(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#ssid
    }
    /// Return a mutable reference to `ssid`
    #[inline]
    pub fn mut_ssid(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#ssid
    }
    /// Set the value of `ssid`
    #[inline]
    pub fn set_ssid(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#ssid = value.into();
        self
    }
    /// Builder method that sets the value of `ssid`. Useful for initializing the message.
    #[inline]
    pub fn init_ssid(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#ssid = value.into();
        self
    }
    /// Return a reference to `bssid`
    #[inline]
    pub fn r#bssid(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#bssid
    }
    /// Return a mutable reference to `bssid`
    #[inline]
    pub fn mut_bssid(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#bssid
    }
    /// Set the value of `bssid`
    #[inline]
    pub fn set_bssid(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#bssid = value.into();
        self
    }
    /// Builder method that sets the value of `bssid`. Useful for initializing the message.
    #[inline]
    pub fn init_bssid(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#bssid = value.into();
        self
    }
    /// Return a reference to `channel`
    #[inline]
    pub fn r#channel(&self) -> &i32 {
        &self.r#channel
    }
    /// Return a mutable reference to `channel`
    #[inline]
    pub fn mut_channel(&mut self) -> &mut i32 {
        &mut self.r#channel
    }
    /// Set the value of `channel`
    #[inline]
    pub fn set_channel(&mut self, value: i32) -> &mut Self {
        self.r#channel = value.into();
        self
    }
    /// Builder method that sets the value of `channel`. Useful for initializing the message.
    #[inline]
    pub fn init_channel(mut self, value: i32) -> Self {
        self.r#channel = value.into();
        self
    }
}
impl ::micropb::MessageDecode for WifiConnectedState {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#ip4_addr;
                    {
                        decoder.decode_string(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                2u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#auth_mode;
                    {
                        let val = decoder.decode_int32().map(|n| WifiAuthMode(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                3u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#ssid;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                4u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#bssid;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                5u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#channel;
                    {
                        let val = decoder.decode_int32()?;
                        let val_ref = &val;
                        if *val_ref != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for WifiConnectedState {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(
            ::core::result::Result::Ok(WifiAuthMode::_MAX_SIZE),
            |size| size + 1usize
        ) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(10usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#ip4_addr;
            if !val_ref.is_empty() {
                encoder.encode_varint32(10u32)?;
                encoder.encode_string(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#auth_mode;
            if val_ref.0 != 0 {
                encoder.encode_varint32(16u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        {
            let val_ref = &self.r#ssid;
            if !val_ref.is_empty() {
                encoder.encode_varint32(26u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#bssid;
            if !val_ref.is_empty() {
                encoder.encode_varint32(34u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#channel;
            if *val_ref != 0 {
                encoder.encode_varint32(40u32)?;
                encoder.encode_int32(*val_ref as _)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#ip4_addr;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#auth_mode;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        {
            let val_ref = &self.r#ssid;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#bssid;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#channel;
            if *val_ref != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(*val_ref as _);
            }
        }
        size
    }
}
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct WifiStationState(pub i32);
impl WifiStationState {
    /// Maximum encoded size of the enum
    pub const _MAX_SIZE: usize = 10usize;
    pub const Connected: Self = Self(0);
    pub const Connecting: Self = Self(1);
    pub const Disconnected: Self = Self(2);
    pub const ConnectionFailed: Self = Self(3);
}
impl core::default::Default for WifiStationState {
    fn default() -> Self {
        Self(0)
    }
}
impl core::convert::From<i32> for WifiStationState {
    fn from(val: i32) -> Self {
        Self(val)
    }
}
impl core::fmt::Debug for WifiStationState {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            WifiStationState::Connected => formatter.write_str("Connected"),
            WifiStationState::Connecting => formatter.write_str("Connecting"),
            WifiStationState::Disconnected => formatter.write_str("Disconnected"),
            WifiStationState::ConnectionFailed => formatter.write_str("ConnectionFailed"),
            Self(n) => formatter.debug_tuple("_Unknown").field(&n).finish(),
        }
    }
}
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct WifiConnectFailedReason(pub i32);
impl WifiConnectFailedReason {
    /// Maximum encoded size of the enum
    pub const _MAX_SIZE: usize = 10usize;
    pub const AuthError: Self = Self(0);
    pub const NetworkNotFound: Self = Self(1);
}
impl core::default::Default for WifiConnectFailedReason {
    fn default() -> Self {
        Self(0)
    }
}
impl core::convert::From<i32> for WifiConnectFailedReason {
    fn from(val: i32) -> Self {
        Self(val)
    }
}
impl core::fmt::Debug for WifiConnectFailedReason {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            WifiConnectFailedReason::AuthError => formatter.write_str("AuthError"),
            WifiConnectFailedReason::NetworkNotFound => formatter.write_str("NetworkNotFound"),
            Self(n) => formatter.debug_tuple("_Unknown").field(&n).finish(),
        }
    }
}
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct WifiAuthMode(pub i32);
impl WifiAuthMode {
    /// Maximum encoded size of the enum
    pub const _MAX_SIZE: usize = 10usize;
    pub const Open: Self = Self(0);
    pub const Wep: Self = Self(1);
    pub const WpaPsk: Self = Self(2);
    pub const Wpa2Psk: Self = Self(3);
    pub const WpaWpa2Psk: Self = Self(4);
    pub const Wpa2Enterprise: Self = Self(5);
    pub const Wpa3Psk: Self = Self(6);
    pub const Wpa2Wpa3Psk: Self = Self(7);
}
impl core::default::Default for WifiAuthMode {
    fn default() -> Self {
        Self(0)
    }
}
impl core::convert::From<i32> for WifiAuthMode {
    fn from(val: i32) -> Self {
        Self(val)
    }
}
impl core::fmt::Debug for WifiAuthMode {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            WifiAuthMode::Open => formatter.write_str("Open"),
            WifiAuthMode::Wep => formatter.write_str("Wep"),
            WifiAuthMode::WpaPsk => formatter.write_str("WpaPsk"),
            WifiAuthMode::Wpa2Psk => formatter.write_str("Wpa2Psk"),
            WifiAuthMode::WpaWpa2Psk => formatter.write_str("WpaWpa2Psk"),
            WifiAuthMode::Wpa2Enterprise => formatter.write_str("Wpa2Enterprise"),
            WifiAuthMode::Wpa3Psk => formatter.write_str("Wpa3Psk"),
            WifiAuthMode::Wpa2Wpa3Psk => formatter.write_str("Wpa2Wpa3Psk"),
            Self(n) => formatter.debug_tuple("_Unknown").field(&n).finish(),
        }
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct CmdGetStatus {}
impl CmdGetStatus {}
impl ::micropb::MessageDecode for CmdGetStatus {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for CmdGetStatus {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone)]
pub struct RespGetStatus {
    pub r#status: Status,
    pub r#sta_state: WifiStationState,
    pub r#state: ::core::option::Option<RespGetStatus_::State>,
}
impl RespGetStatus {
    /// Return a reference to `status`
    #[inline]
    pub fn r#status(&self) -> &Status {
        &self.r#status
    }
    /// Return a mutable reference to `status`
    #[inline]
    pub fn mut_status(&mut self) -> &mut Status {
        &mut self.r#status
    }
    /// Set the value of `status`
    #[inline]
    pub fn set_status(&mut self, value: Status) -> &mut Self {
        self.r#status = value.into();
        self
    }
    /// Builder method that sets the value of `status`. Useful for initializing the message.
    #[inline]
    pub fn init_status(mut self, value: Status) -> Self {
        self.r#status = value.into();
        self
    }
    /// Return a reference to `sta_state`
    #[inline]
    pub fn r#sta_state(&self) -> &WifiStationState {
        &self.r#sta_state
    }
    /// Return a mutable reference to `sta_state`
    #[inline]
    pub fn mut_sta_state(&mut self) -> &mut WifiStationState {
        &mut self.r#sta_state
    }
    /// Set the value of `sta_state`
    #[inline]
    pub fn set_sta_state(&mut self, value: WifiStationState) -> &mut Self {
        self.r#sta_state = value.into();
        self
    }
    /// Builder method that sets the value of `sta_state`. Useful for initializing the message.
    #[inline]
    pub fn init_sta_state(mut self, value: WifiStationState) -> Self {
        self.r#sta_state = value.into();
        self
    }
}
impl ::micropb::MessageDecode for RespGetStatus {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#status;
                    {
                        let val = decoder.decode_int32().map(|n| Status(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                2u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#sta_state;
                    {
                        let val = decoder.decode_int32().map(|n| WifiStationState(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                10u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#state {
                            if let RespGetStatus_::State::FailReason(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#state = ::core::option::Option::Some(
                            RespGetStatus_::State::FailReason(::core::default::Default::default()),
                        );
                    };
                    let val = decoder
                        .decode_int32()
                        .map(|n| WifiConnectFailedReason(n as _))?;
                    *mut_ref = val as _;
                }
                11u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#state {
                            if let RespGetStatus_::State::Connected(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#state = ::core::option::Option::Some(
                            RespGetStatus_::State::Connected(::core::default::Default::default()),
                        );
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                12u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#state {
                            if let RespGetStatus_::State::AttemptFailed(variant) = &mut *variant {
                                break &mut *variant;
                            }
                        }
                        self.r#state =
                            ::core::option::Option::Some(RespGetStatus_::State::AttemptFailed(
                                ::core::default::Default::default(),
                            ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for RespGetStatus {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(Status::_MAX_SIZE), |size| size
            + 1usize)
        {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(
            ::core::result::Result::Ok(WifiStationState::_MAX_SIZE),
            |size| size + 1usize
        ) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match 'oneof: {
            let mut max_size = 0;
            match ::micropb::const_map!(
                ::core::result::Result::Ok(WifiConnectFailedReason::_MAX_SIZE),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <WifiConnectedState as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <WifiAttemptFailed as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            ::core::result::Result::Ok(max_size)
        } {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        {
            let val_ref = &self.r#sta_state;
            if val_ref.0 != 0 {
                encoder.encode_varint32(16u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        if let Some(oneof) = &self.r#state {
            match &*oneof {
                RespGetStatus_::State::FailReason(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(80u32)?;
                    encoder.encode_int32(val_ref.0 as _)?;
                }
                RespGetStatus_::State::Connected(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(90u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                RespGetStatus_::State::AttemptFailed(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(98u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        {
            let val_ref = &self.r#sta_state;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        if let Some(oneof) = &self.r#state {
            match &*oneof {
                RespGetStatus_::State::FailReason(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
                }
                RespGetStatus_::State::Connected(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                RespGetStatus_::State::AttemptFailed(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
            }
        }
        size
    }
}
/// Inner types for `RespGetStatus`
pub mod RespGetStatus_ {
    #[derive(Debug, PartialEq, Clone)]
    pub enum State {
        FailReason(super::WifiConnectFailedReason),
        Connected(super::WifiConnectedState),
        AttemptFailed(super::WifiAttemptFailed),
    }
}
#[derive(Debug, Default, PartialEq, Clone)]
pub struct CmdSetConfig {
    pub r#ssid: ::heapless::Vec<u8, 64>,
    pub r#passphrase: ::heapless::Vec<u8, 64>,
    pub r#bssid: ::heapless::Vec<u8, 64>,
    pub r#channel: i32,
}
impl CmdSetConfig {
    /// Return a reference to `ssid`
    #[inline]
    pub fn r#ssid(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#ssid
    }
    /// Return a mutable reference to `ssid`
    #[inline]
    pub fn mut_ssid(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#ssid
    }
    /// Set the value of `ssid`
    #[inline]
    pub fn set_ssid(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#ssid = value.into();
        self
    }
    /// Builder method that sets the value of `ssid`. Useful for initializing the message.
    #[inline]
    pub fn init_ssid(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#ssid = value.into();
        self
    }
    /// Return a reference to `passphrase`
    #[inline]
    pub fn r#passphrase(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#passphrase
    }
    /// Return a mutable reference to `passphrase`
    #[inline]
    pub fn mut_passphrase(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#passphrase
    }
    /// Set the value of `passphrase`
    #[inline]
    pub fn set_passphrase(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#passphrase = value.into();
        self
    }
    /// Builder method that sets the value of `passphrase`. Useful for initializing the message.
    #[inline]
    pub fn init_passphrase(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#passphrase = value.into();
        self
    }
    /// Return a reference to `bssid`
    #[inline]
    pub fn r#bssid(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#bssid
    }
    /// Return a mutable reference to `bssid`
    #[inline]
    pub fn mut_bssid(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#bssid
    }
    /// Set the value of `bssid`
    #[inline]
    pub fn set_bssid(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#bssid = value.into();
        self
    }
    /// Builder method that sets the value of `bssid`. Useful for initializing the message.
    #[inline]
    pub fn init_bssid(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#bssid = value.into();
        self
    }
    /// Return a reference to `channel`
    #[inline]
    pub fn r#channel(&self) -> &i32 {
        &self.r#channel
    }
    /// Return a mutable reference to `channel`
    #[inline]
    pub fn mut_channel(&mut self) -> &mut i32 {
        &mut self.r#channel
    }
    /// Set the value of `channel`
    #[inline]
    pub fn set_channel(&mut self, value: i32) -> &mut Self {
        self.r#channel = value.into();
        self
    }
    /// Builder method that sets the value of `channel`. Useful for initializing the message.
    #[inline]
    pub fn init_channel(mut self, value: i32) -> Self {
        self.r#channel = value.into();
        self
    }
}
impl ::micropb::MessageDecode for CmdSetConfig {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#ssid;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                2u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#passphrase;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                3u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#bssid;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                4u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#channel;
                    {
                        let val = decoder.decode_int32()?;
                        let val_ref = &val;
                        if *val_ref != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for CmdSetConfig {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(10usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#ssid;
            if !val_ref.is_empty() {
                encoder.encode_varint32(10u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#passphrase;
            if !val_ref.is_empty() {
                encoder.encode_varint32(18u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#bssid;
            if !val_ref.is_empty() {
                encoder.encode_varint32(26u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#channel;
            if *val_ref != 0 {
                encoder.encode_varint32(32u32)?;
                encoder.encode_int32(*val_ref as _)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#ssid;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#passphrase;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#bssid;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#channel;
            if *val_ref != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(*val_ref as _);
            }
        }
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct RespSetConfig {
    pub r#status: Status,
}
impl RespSetConfig {
    /// Return a reference to `status`
    #[inline]
    pub fn r#status(&self) -> &Status {
        &self.r#status
    }
    /// Return a mutable reference to `status`
    #[inline]
    pub fn mut_status(&mut self) -> &mut Status {
        &mut self.r#status
    }
    /// Set the value of `status`
    #[inline]
    pub fn set_status(&mut self, value: Status) -> &mut Self {
        self.r#status = value.into();
        self
    }
    /// Builder method that sets the value of `status`. Useful for initializing the message.
    #[inline]
    pub fn init_status(mut self, value: Status) -> Self {
        self.r#status = value.into();
        self
    }
}
impl ::micropb::MessageDecode for RespSetConfig {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#status;
                    {
                        let val = decoder.decode_int32().map(|n| Status(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for RespSetConfig {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(Status::_MAX_SIZE), |size| size
            + 1usize)
        {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct CmdApplyConfig {}
impl CmdApplyConfig {}
impl ::micropb::MessageDecode for CmdApplyConfig {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for CmdApplyConfig {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct RespApplyConfig {
    pub r#status: Status,
}
impl RespApplyConfig {
    /// Return a reference to `status`
    #[inline]
    pub fn r#status(&self) -> &Status {
        &self.r#status
    }
    /// Return a mutable reference to `status`
    #[inline]
    pub fn mut_status(&mut self) -> &mut Status {
        &mut self.r#status
    }
    /// Set the value of `status`
    #[inline]
    pub fn set_status(&mut self, value: Status) -> &mut Self {
        self.r#status = value.into();
        self
    }
    /// Builder method that sets the value of `status`. Useful for initializing the message.
    #[inline]
    pub fn init_status(mut self, value: Status) -> Self {
        self.r#status = value.into();
        self
    }
}
impl ::micropb::MessageDecode for RespApplyConfig {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#status;
                    {
                        let val = decoder.decode_int32().map(|n| Status(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for RespApplyConfig {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(Status::_MAX_SIZE), |size| size
            + 1usize)
        {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone)]
pub struct WiFiConfigPayload {
    pub r#msg: WiFiConfigMsgType,
    pub r#payload: ::core::option::Option<WiFiConfigPayload_::Payload>,
}
impl WiFiConfigPayload {
    /// Return a reference to `msg`
    #[inline]
    pub fn r#msg(&self) -> &WiFiConfigMsgType {
        &self.r#msg
    }
    /// Return a mutable reference to `msg`
    #[inline]
    pub fn mut_msg(&mut self) -> &mut WiFiConfigMsgType {
        &mut self.r#msg
    }
    /// Set the value of `msg`
    #[inline]
    pub fn set_msg(&mut self, value: WiFiConfigMsgType) -> &mut Self {
        self.r#msg = value.into();
        self
    }
    /// Builder method that sets the value of `msg`. Useful for initializing the message.
    #[inline]
    pub fn init_msg(mut self, value: WiFiConfigMsgType) -> Self {
        self.r#msg = value.into();
        self
    }
}
impl ::micropb::MessageDecode for WiFiConfigPayload {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#msg;
                    {
                        let val = decoder.decode_int32().map(|n| WiFiConfigMsgType(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                10u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiConfigPayload_::Payload::CmdGetStatus(variant) =
                                &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(
                            WiFiConfigPayload_::Payload::CmdGetStatus(
                                ::core::default::Default::default(),
                            ),
                        );
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                11u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiConfigPayload_::Payload::RespGetStatus(variant) =
                                &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(
                            WiFiConfigPayload_::Payload::RespGetStatus(
                                ::core::default::Default::default(),
                            ),
                        );
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                12u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiConfigPayload_::Payload::CmdSetConfig(variant) =
                                &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(
                            WiFiConfigPayload_::Payload::CmdSetConfig(
                                ::core::default::Default::default(),
                            ),
                        );
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                13u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiConfigPayload_::Payload::RespSetConfig(variant) =
                                &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(
                            WiFiConfigPayload_::Payload::RespSetConfig(
                                ::core::default::Default::default(),
                            ),
                        );
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                14u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiConfigPayload_::Payload::CmdApplyConfig(variant) =
                                &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(
                            WiFiConfigPayload_::Payload::CmdApplyConfig(
                                ::core::default::Default::default(),
                            ),
                        );
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                15u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiConfigPayload_::Payload::RespApplyConfig(variant) =
                                &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(
                            WiFiConfigPayload_::Payload::RespApplyConfig(
                                ::core::default::Default::default(),
                            ),
                        );
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for WiFiConfigPayload {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(
            ::core::result::Result::Ok(WiFiConfigMsgType::_MAX_SIZE),
            |size| size + 1usize
        ) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match 'oneof: {
            let mut max_size = 0;
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <CmdGetStatus as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <RespGetStatus as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <CmdSetConfig as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <RespSetConfig as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <CmdApplyConfig as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <RespApplyConfig as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            ::core::result::Result::Ok(max_size)
        } {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#msg;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        if let Some(oneof) = &self.r#payload {
            match &*oneof {
                WiFiConfigPayload_::Payload::CmdGetStatus(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(82u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiConfigPayload_::Payload::RespGetStatus(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(90u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiConfigPayload_::Payload::CmdSetConfig(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(98u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiConfigPayload_::Payload::RespSetConfig(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(106u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiConfigPayload_::Payload::CmdApplyConfig(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(114u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiConfigPayload_::Payload::RespApplyConfig(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(122u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#msg;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        if let Some(oneof) = &self.r#payload {
            match &*oneof {
                WiFiConfigPayload_::Payload::CmdGetStatus(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiConfigPayload_::Payload::RespGetStatus(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiConfigPayload_::Payload::CmdSetConfig(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiConfigPayload_::Payload::RespSetConfig(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiConfigPayload_::Payload::CmdApplyConfig(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiConfigPayload_::Payload::RespApplyConfig(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
            }
        }
        size
    }
}
/// Inner types for `WiFiConfigPayload`
pub mod WiFiConfigPayload_ {
    #[derive(Debug, PartialEq, Clone)]
    pub enum Payload {
        CmdGetStatus(super::CmdGetStatus),
        RespGetStatus(super::RespGetStatus),
        CmdSetConfig(super::CmdSetConfig),
        RespSetConfig(super::RespSetConfig),
        CmdApplyConfig(super::CmdApplyConfig),
        RespApplyConfig(super::RespApplyConfig),
    }
}
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct WiFiConfigMsgType(pub i32);
impl WiFiConfigMsgType {
    /// Maximum encoded size of the enum
    pub const _MAX_SIZE: usize = 10usize;
    pub const TypeCmdGetStatus: Self = Self(0);
    pub const TypeRespGetStatus: Self = Self(1);
    pub const TypeCmdSetConfig: Self = Self(2);
    pub const TypeRespSetConfig: Self = Self(3);
    pub const TypeCmdApplyConfig: Self = Self(4);
    pub const TypeRespApplyConfig: Self = Self(5);
}
impl core::default::Default for WiFiConfigMsgType {
    fn default() -> Self {
        Self(0)
    }
}
impl core::convert::From<i32> for WiFiConfigMsgType {
    fn from(val: i32) -> Self {
        Self(val)
    }
}
impl core::fmt::Debug for WiFiConfigMsgType {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            WiFiConfigMsgType::TypeCmdGetStatus => formatter.write_str("TypeCmdGetStatus"),
            WiFiConfigMsgType::TypeRespGetStatus => formatter.write_str("TypeRespGetStatus"),
            WiFiConfigMsgType::TypeCmdSetConfig => formatter.write_str("TypeCmdSetConfig"),
            WiFiConfigMsgType::TypeRespSetConfig => formatter.write_str("TypeRespSetConfig"),
            WiFiConfigMsgType::TypeCmdApplyConfig => formatter.write_str("TypeCmdApplyConfig"),
            WiFiConfigMsgType::TypeRespApplyConfig => formatter.write_str("TypeRespApplyConfig"),
            Self(n) => formatter.debug_tuple("_Unknown").field(&n).finish(),
        }
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct CmdScanStart {
    pub r#blocking: bool,
    pub r#passive: bool,
    pub r#group_channels: u32,
    pub r#period_ms: u32,
}
impl CmdScanStart {
    /// Return a reference to `blocking`
    #[inline]
    pub fn r#blocking(&self) -> &bool {
        &self.r#blocking
    }
    /// Return a mutable reference to `blocking`
    #[inline]
    pub fn mut_blocking(&mut self) -> &mut bool {
        &mut self.r#blocking
    }
    /// Set the value of `blocking`
    #[inline]
    pub fn set_blocking(&mut self, value: bool) -> &mut Self {
        self.r#blocking = value.into();
        self
    }
    /// Builder method that sets the value of `blocking`. Useful for initializing the message.
    #[inline]
    pub fn init_blocking(mut self, value: bool) -> Self {
        self.r#blocking = value.into();
        self
    }
    /// Return a reference to `passive`
    #[inline]
    pub fn r#passive(&self) -> &bool {
        &self.r#passive
    }
    /// Return a mutable reference to `passive`
    #[inline]
    pub fn mut_passive(&mut self) -> &mut bool {
        &mut self.r#passive
    }
    /// Set the value of `passive`
    #[inline]
    pub fn set_passive(&mut self, value: bool) -> &mut Self {
        self.r#passive = value.into();
        self
    }
    /// Builder method that sets the value of `passive`. Useful for initializing the message.
    #[inline]
    pub fn init_passive(mut self, value: bool) -> Self {
        self.r#passive = value.into();
        self
    }
    /// Return a reference to `group_channels`
    #[inline]
    pub fn r#group_channels(&self) -> &u32 {
        &self.r#group_channels
    }
    /// Return a mutable reference to `group_channels`
    #[inline]
    pub fn mut_group_channels(&mut self) -> &mut u32 {
        &mut self.r#group_channels
    }
    /// Set the value of `group_channels`
    #[inline]
    pub fn set_group_channels(&mut self, value: u32) -> &mut Self {
        self.r#group_channels = value.into();
        self
    }
    /// Builder method that sets the value of `group_channels`. Useful for initializing the message.
    #[inline]
    pub fn init_group_channels(mut self, value: u32) -> Self {
        self.r#group_channels = value.into();
        self
    }
    /// Return a reference to `period_ms`
    #[inline]
    pub fn r#period_ms(&self) -> &u32 {
        &self.r#period_ms
    }
    /// Return a mutable reference to `period_ms`
    #[inline]
    pub fn mut_period_ms(&mut self) -> &mut u32 {
        &mut self.r#period_ms
    }
    /// Set the value of `period_ms`
    #[inline]
    pub fn set_period_ms(&mut self, value: u32) -> &mut Self {
        self.r#period_ms = value.into();
        self
    }
    /// Builder method that sets the value of `period_ms`. Useful for initializing the message.
    #[inline]
    pub fn init_period_ms(mut self, value: u32) -> Self {
        self.r#period_ms = value.into();
        self
    }
}
impl ::micropb::MessageDecode for CmdScanStart {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#blocking;
                    {
                        let val = decoder.decode_bool()?;
                        let val_ref = &val;
                        if *val_ref {
                            *mut_ref = val as _;
                        }
                    };
                }
                2u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#passive;
                    {
                        let val = decoder.decode_bool()?;
                        let val_ref = &val;
                        if *val_ref {
                            *mut_ref = val as _;
                        }
                    };
                }
                3u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#group_channels;
                    {
                        let val = decoder.decode_varint32()?;
                        let val_ref = &val;
                        if *val_ref != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                4u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#period_ms;
                    {
                        let val = decoder.decode_varint32()?;
                        let val_ref = &val;
                        if *val_ref != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for CmdScanStart {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(1usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(1usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(5usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(5usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#blocking;
            if *val_ref {
                encoder.encode_varint32(8u32)?;
                encoder.encode_bool(*val_ref)?;
            }
        }
        {
            let val_ref = &self.r#passive;
            if *val_ref {
                encoder.encode_varint32(16u32)?;
                encoder.encode_bool(*val_ref)?;
            }
        }
        {
            let val_ref = &self.r#group_channels;
            if *val_ref != 0 {
                encoder.encode_varint32(24u32)?;
                encoder.encode_varint32(*val_ref as _)?;
            }
        }
        {
            let val_ref = &self.r#period_ms;
            if *val_ref != 0 {
                encoder.encode_varint32(32u32)?;
                encoder.encode_varint32(*val_ref as _)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#blocking;
            if *val_ref {
                size += 1usize + 1;
            }
        }
        {
            let val_ref = &self.r#passive;
            if *val_ref {
                size += 1usize + 1;
            }
        }
        {
            let val_ref = &self.r#group_channels;
            if *val_ref != 0 {
                size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
            }
        }
        {
            let val_ref = &self.r#period_ms;
            if *val_ref != 0 {
                size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
            }
        }
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct RespScanStart {}
impl RespScanStart {}
impl ::micropb::MessageDecode for RespScanStart {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for RespScanStart {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct CmdScanStatus {}
impl CmdScanStatus {}
impl ::micropb::MessageDecode for CmdScanStatus {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for CmdScanStatus {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct RespScanStatus {
    pub r#scan_finished: bool,
    pub r#result_count: u32,
}
impl RespScanStatus {
    /// Return a reference to `scan_finished`
    #[inline]
    pub fn r#scan_finished(&self) -> &bool {
        &self.r#scan_finished
    }
    /// Return a mutable reference to `scan_finished`
    #[inline]
    pub fn mut_scan_finished(&mut self) -> &mut bool {
        &mut self.r#scan_finished
    }
    /// Set the value of `scan_finished`
    #[inline]
    pub fn set_scan_finished(&mut self, value: bool) -> &mut Self {
        self.r#scan_finished = value.into();
        self
    }
    /// Builder method that sets the value of `scan_finished`. Useful for initializing the message.
    #[inline]
    pub fn init_scan_finished(mut self, value: bool) -> Self {
        self.r#scan_finished = value.into();
        self
    }
    /// Return a reference to `result_count`
    #[inline]
    pub fn r#result_count(&self) -> &u32 {
        &self.r#result_count
    }
    /// Return a mutable reference to `result_count`
    #[inline]
    pub fn mut_result_count(&mut self) -> &mut u32 {
        &mut self.r#result_count
    }
    /// Set the value of `result_count`
    #[inline]
    pub fn set_result_count(&mut self, value: u32) -> &mut Self {
        self.r#result_count = value.into();
        self
    }
    /// Builder method that sets the value of `result_count`. Useful for initializing the message.
    #[inline]
    pub fn init_result_count(mut self, value: u32) -> Self {
        self.r#result_count = value.into();
        self
    }
}
impl ::micropb::MessageDecode for RespScanStatus {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#scan_finished;
                    {
                        let val = decoder.decode_bool()?;
                        let val_ref = &val;
                        if *val_ref {
                            *mut_ref = val as _;
                        }
                    };
                }
                2u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#result_count;
                    {
                        let val = decoder.decode_varint32()?;
                        let val_ref = &val;
                        if *val_ref != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for RespScanStatus {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(1usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(5usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#scan_finished;
            if *val_ref {
                encoder.encode_varint32(8u32)?;
                encoder.encode_bool(*val_ref)?;
            }
        }
        {
            let val_ref = &self.r#result_count;
            if *val_ref != 0 {
                encoder.encode_varint32(16u32)?;
                encoder.encode_varint32(*val_ref as _)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#scan_finished;
            if *val_ref {
                size += 1usize + 1;
            }
        }
        {
            let val_ref = &self.r#result_count;
            if *val_ref != 0 {
                size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
            }
        }
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct CmdScanResult {
    pub r#start_index: u32,
    pub r#count: u32,
}
impl CmdScanResult {
    /// Return a reference to `start_index`
    #[inline]
    pub fn r#start_index(&self) -> &u32 {
        &self.r#start_index
    }
    /// Return a mutable reference to `start_index`
    #[inline]
    pub fn mut_start_index(&mut self) -> &mut u32 {
        &mut self.r#start_index
    }
    /// Set the value of `start_index`
    #[inline]
    pub fn set_start_index(&mut self, value: u32) -> &mut Self {
        self.r#start_index = value.into();
        self
    }
    /// Builder method that sets the value of `start_index`. Useful for initializing the message.
    #[inline]
    pub fn init_start_index(mut self, value: u32) -> Self {
        self.r#start_index = value.into();
        self
    }
    /// Return a reference to `count`
    #[inline]
    pub fn r#count(&self) -> &u32 {
        &self.r#count
    }
    /// Return a mutable reference to `count`
    #[inline]
    pub fn mut_count(&mut self) -> &mut u32 {
        &mut self.r#count
    }
    /// Set the value of `count`
    #[inline]
    pub fn set_count(&mut self, value: u32) -> &mut Self {
        self.r#count = value.into();
        self
    }
    /// Builder method that sets the value of `count`. Useful for initializing the message.
    #[inline]
    pub fn init_count(mut self, value: u32) -> Self {
        self.r#count = value.into();
        self
    }
}
impl ::micropb::MessageDecode for CmdScanResult {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#start_index;
                    {
                        let val = decoder.decode_varint32()?;
                        let val_ref = &val;
                        if *val_ref != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                2u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#count;
                    {
                        let val = decoder.decode_varint32()?;
                        let val_ref = &val;
                        if *val_ref != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for CmdScanResult {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(5usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(5usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#start_index;
            if *val_ref != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_varint32(*val_ref as _)?;
            }
        }
        {
            let val_ref = &self.r#count;
            if *val_ref != 0 {
                encoder.encode_varint32(16u32)?;
                encoder.encode_varint32(*val_ref as _)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#start_index;
            if *val_ref != 0 {
                size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
            }
        }
        {
            let val_ref = &self.r#count;
            if *val_ref != 0 {
                size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
            }
        }
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone)]
pub struct WiFiScanResult {
    pub r#ssid: ::heapless::Vec<u8, 64>,
    pub r#channel: u32,
    pub r#rssi: i32,
    pub r#bssid: ::heapless::Vec<u8, 64>,
    pub r#auth: WifiAuthMode,
}
impl WiFiScanResult {
    /// Return a reference to `ssid`
    #[inline]
    pub fn r#ssid(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#ssid
    }
    /// Return a mutable reference to `ssid`
    #[inline]
    pub fn mut_ssid(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#ssid
    }
    /// Set the value of `ssid`
    #[inline]
    pub fn set_ssid(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#ssid = value.into();
        self
    }
    /// Builder method that sets the value of `ssid`. Useful for initializing the message.
    #[inline]
    pub fn init_ssid(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#ssid = value.into();
        self
    }
    /// Return a reference to `channel`
    #[inline]
    pub fn r#channel(&self) -> &u32 {
        &self.r#channel
    }
    /// Return a mutable reference to `channel`
    #[inline]
    pub fn mut_channel(&mut self) -> &mut u32 {
        &mut self.r#channel
    }
    /// Set the value of `channel`
    #[inline]
    pub fn set_channel(&mut self, value: u32) -> &mut Self {
        self.r#channel = value.into();
        self
    }
    /// Builder method that sets the value of `channel`. Useful for initializing the message.
    #[inline]
    pub fn init_channel(mut self, value: u32) -> Self {
        self.r#channel = value.into();
        self
    }
    /// Return a reference to `rssi`
    #[inline]
    pub fn r#rssi(&self) -> &i32 {
        &self.r#rssi
    }
    /// Return a mutable reference to `rssi`
    #[inline]
    pub fn mut_rssi(&mut self) -> &mut i32 {
        &mut self.r#rssi
    }
    /// Set the value of `rssi`
    #[inline]
    pub fn set_rssi(&mut self, value: i32) -> &mut Self {
        self.r#rssi = value.into();
        self
    }
    /// Builder method that sets the value of `rssi`. Useful for initializing the message.
    #[inline]
    pub fn init_rssi(mut self, value: i32) -> Self {
        self.r#rssi = value.into();
        self
    }
    /// Return a reference to `bssid`
    #[inline]
    pub fn r#bssid(&self) -> &::heapless::Vec<u8, 64> {
        &self.r#bssid
    }
    /// Return a mutable reference to `bssid`
    #[inline]
    pub fn mut_bssid(&mut self) -> &mut ::heapless::Vec<u8, 64> {
        &mut self.r#bssid
    }
    /// Set the value of `bssid`
    #[inline]
    pub fn set_bssid(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
        self.r#bssid = value.into();
        self
    }
    /// Builder method that sets the value of `bssid`. Useful for initializing the message.
    #[inline]
    pub fn init_bssid(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
        self.r#bssid = value.into();
        self
    }
    /// Return a reference to `auth`
    #[inline]
    pub fn r#auth(&self) -> &WifiAuthMode {
        &self.r#auth
    }
    /// Return a mutable reference to `auth`
    #[inline]
    pub fn mut_auth(&mut self) -> &mut WifiAuthMode {
        &mut self.r#auth
    }
    /// Set the value of `auth`
    #[inline]
    pub fn set_auth(&mut self, value: WifiAuthMode) -> &mut Self {
        self.r#auth = value.into();
        self
    }
    /// Builder method that sets the value of `auth`. Useful for initializing the message.
    #[inline]
    pub fn init_auth(mut self, value: WifiAuthMode) -> Self {
        self.r#auth = value.into();
        self
    }
}
impl ::micropb::MessageDecode for WiFiScanResult {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#ssid;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                2u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#channel;
                    {
                        let val = decoder.decode_varint32()?;
                        let val_ref = &val;
                        if *val_ref != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                3u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#rssi;
                    {
                        let val = decoder.decode_int32()?;
                        let val_ref = &val;
                        if *val_ref != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                4u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#bssid;
                    {
                        decoder.decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                    };
                }
                5u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#auth;
                    {
                        let val = decoder.decode_int32().map(|n| WifiAuthMode(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for WiFiScanResult {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(5usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(10usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(65usize), |size| size + 1usize) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(
            ::core::result::Result::Ok(WifiAuthMode::_MAX_SIZE),
            |size| size + 1usize
        ) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#ssid;
            if !val_ref.is_empty() {
                encoder.encode_varint32(10u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#channel;
            if *val_ref != 0 {
                encoder.encode_varint32(16u32)?;
                encoder.encode_varint32(*val_ref as _)?;
            }
        }
        {
            let val_ref = &self.r#rssi;
            if *val_ref != 0 {
                encoder.encode_varint32(24u32)?;
                encoder.encode_int32(*val_ref as _)?;
            }
        }
        {
            let val_ref = &self.r#bssid;
            if !val_ref.is_empty() {
                encoder.encode_varint32(34u32)?;
                encoder.encode_bytes(val_ref)?;
            }
        }
        {
            let val_ref = &self.r#auth;
            if val_ref.0 != 0 {
                encoder.encode_varint32(40u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#ssid;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#channel;
            if *val_ref != 0 {
                size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
            }
        }
        {
            let val_ref = &self.r#rssi;
            if *val_ref != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(*val_ref as _);
            }
        }
        {
            let val_ref = &self.r#bssid;
            if !val_ref.is_empty() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
            }
        }
        {
            let val_ref = &self.r#auth;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone)]
pub struct RespScanResult {
    pub r#entries: ::heapless::Vec<WiFiScanResult, 16>,
}
impl RespScanResult {}
impl ::micropb::MessageDecode for RespScanResult {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 2u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut val: WiFiScanResult = ::core::default::Default::default();
                    let mut_ref = &mut val;
                    {
                        decoder.decode_submessage_delimited(mut_ref)?;
                    };
                    if let (Err(_), false) =
                        (self.r#entries.pb_push(val), decoder.ignore_repeated_cap_err)
                    {
                        return Err(::micropb::DecodeError::Capacity);
                    }
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for RespScanResult {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(
            ::micropb::const_map!(
                <WiFiScanResult as ::micropb::MessageEncode>::MAX_SIZE,
                |size| ::micropb::size::sizeof_len_record(size)
            ),
            |size| (size + 1usize) * 16usize
        ) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            for (i, val_ref) in self.r#entries.iter().enumerate() {
                encoder.encode_varint32(10u32)?;
                val_ref.encode_len_delimited(encoder)?;
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            for (i, val_ref) in self.r#entries.iter().enumerate() {
                size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
            }
        }
        size
    }
}
#[derive(Debug, Default, PartialEq, Clone)]
pub struct WiFiScanPayload {
    pub r#msg: WiFiScanMsgType,
    pub r#status: Status,
    pub r#payload: ::core::option::Option<WiFiScanPayload_::Payload>,
}
impl WiFiScanPayload {
    /// Return a reference to `msg`
    #[inline]
    pub fn r#msg(&self) -> &WiFiScanMsgType {
        &self.r#msg
    }
    /// Return a mutable reference to `msg`
    #[inline]
    pub fn mut_msg(&mut self) -> &mut WiFiScanMsgType {
        &mut self.r#msg
    }
    /// Set the value of `msg`
    #[inline]
    pub fn set_msg(&mut self, value: WiFiScanMsgType) -> &mut Self {
        self.r#msg = value.into();
        self
    }
    /// Builder method that sets the value of `msg`. Useful for initializing the message.
    #[inline]
    pub fn init_msg(mut self, value: WiFiScanMsgType) -> Self {
        self.r#msg = value.into();
        self
    }
    /// Return a reference to `status`
    #[inline]
    pub fn r#status(&self) -> &Status {
        &self.r#status
    }
    /// Return a mutable reference to `status`
    #[inline]
    pub fn mut_status(&mut self) -> &mut Status {
        &mut self.r#status
    }
    /// Set the value of `status`
    #[inline]
    pub fn set_status(&mut self, value: Status) -> &mut Self {
        self.r#status = value.into();
        self
    }
    /// Builder method that sets the value of `status`. Useful for initializing the message.
    #[inline]
    pub fn init_status(mut self, value: Status) -> Self {
        self.r#status = value.into();
        self
    }
}
impl ::micropb::MessageDecode for WiFiScanPayload {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        use ::micropb::{FieldDecode, PbBytes, PbMap, PbString, PbVec};
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#msg;
                    {
                        let val = decoder.decode_int32().map(|n| WiFiScanMsgType(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                2u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let mut_ref = &mut self.r#status;
                    {
                        let val = decoder.decode_int32().map(|n| Status(n as _))?;
                        let val_ref = &val;
                        if val_ref.0 != 0 {
                            *mut_ref = val as _;
                        }
                    };
                }
                10u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiScanPayload_::Payload::CmdScanStart(variant) = &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload =
                            ::core::option::Option::Some(WiFiScanPayload_::Payload::CmdScanStart(
                                ::core::default::Default::default(),
                            ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                11u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiScanPayload_::Payload::RespScanStart(variant) = &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload =
                            ::core::option::Option::Some(WiFiScanPayload_::Payload::RespScanStart(
                                ::core::default::Default::default(),
                            ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                12u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiScanPayload_::Payload::CmdScanStatus(variant) = &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload =
                            ::core::option::Option::Some(WiFiScanPayload_::Payload::CmdScanStatus(
                                ::core::default::Default::default(),
                            ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                13u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiScanPayload_::Payload::RespScanStatus(variant) =
                                &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(
                            WiFiScanPayload_::Payload::RespScanStatus(
                                ::core::default::Default::default(),
                            ),
                        );
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                14u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiScanPayload_::Payload::CmdScanResult(variant) = &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload =
                            ::core::option::Option::Some(WiFiScanPayload_::Payload::CmdScanResult(
                                ::core::default::Default::default(),
                            ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                15u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(variant) = &mut self.r#payload {
                            if let WiFiScanPayload_::Payload::RespScanResult(variant) =
                                &mut *variant
                            {
                                break &mut *variant;
                            }
                        }
                        self.r#payload = ::core::option::Option::Some(
                            WiFiScanPayload_::Payload::RespScanResult(
                                ::core::default::Default::default(),
                            ),
                        );
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for WiFiScanPayload {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
        let mut max_size = 0;
        match ::micropb::const_map!(
            ::core::result::Result::Ok(WiFiScanMsgType::_MAX_SIZE),
            |size| size + 1usize
        ) {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match ::micropb::const_map!(::core::result::Result::Ok(Status::_MAX_SIZE), |size| size
            + 1usize)
        {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        match 'oneof: {
            let mut max_size = 0;
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <CmdScanStart as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <RespScanStart as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <CmdScanStatus as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <RespScanStatus as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <CmdScanResult as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::micropb::const_map!(
                    <RespScanResult as ::micropb::MessageEncode>::MAX_SIZE,
                    |size| ::micropb::size::sizeof_len_record(size)
                ),
                |size| size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    if size > max_size {
                        max_size = size;
                    }
                }
                ::core::result::Result::Err(err) => {
                    break 'oneof (::core::result::Result::<usize, _>::Err(err));
                }
            }
            ::core::result::Result::Ok(max_size)
        } {
            ::core::result::Result::Ok(size) => {
                max_size += size;
            }
            ::core::result::Result::Err(err) => {
                break 'msg (::core::result::Result::<usize, _>::Err(err));
            }
        }
        ::core::result::Result::Ok(max_size)
    };
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        use ::micropb::{FieldEncode, PbMap};
        {
            let val_ref = &self.r#msg;
            if val_ref.0 != 0 {
                encoder.encode_varint32(8u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                encoder.encode_varint32(16u32)?;
                encoder.encode_int32(val_ref.0 as _)?;
            }
        }
        if let Some(oneof) = &self.r#payload {
            match &*oneof {
                WiFiScanPayload_::Payload::CmdScanStart(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(82u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiScanPayload_::Payload::RespScanStart(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(90u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiScanPayload_::Payload::CmdScanStatus(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(98u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiScanPayload_::Payload::RespScanStatus(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(106u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiScanPayload_::Payload::CmdScanResult(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(114u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiScanPayload_::Payload::RespScanResult(val_ref) => {
                    let val_ref = &*val_ref;
                    encoder.encode_varint32(122u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        use ::micropb::{FieldEncode, PbMap};
        let mut size = 0;
        {
            let val_ref = &self.r#msg;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        {
            let val_ref = &self.r#status;
            if val_ref.0 != 0 {
                size += 1usize + ::micropb::size::sizeof_int32(val_ref.0 as _);
            }
        }
        if let Some(oneof) = &self.r#payload {
            match &*oneof {
                WiFiScanPayload_::Payload::CmdScanStart(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiScanPayload_::Payload::RespScanStart(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiScanPayload_::Payload::CmdScanStatus(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiScanPayload_::Payload::RespScanStatus(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiScanPayload_::Payload::CmdScanResult(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiScanPayload_::Payload::RespScanResult(val_ref) => {
                    let val_ref = &*val_ref;
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
            }
        }
        size
    }
}
/// Inner types for `WiFiScanPayload`
pub mod WiFiScanPayload_ {
    #[derive(Debug, PartialEq, Clone)]
    pub enum Payload {
        CmdScanStart(super::CmdScanStart),
        RespScanStart(super::RespScanStart),
        CmdScanStatus(super::CmdScanStatus),
        RespScanStatus(super::RespScanStatus),
        CmdScanResult(super::CmdScanResult),
        RespScanResult(super::RespScanResult),
    }
}
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct WiFiScanMsgType(pub i32);
impl WiFiScanMsgType {
    /// Maximum encoded size of the enum
    pub const _MAX_SIZE: usize = 10usize;
    pub const TypeCmdScanStart: Self = Self(0);
    pub const TypeRespScanStart: Self = Self(1);
    pub const TypeCmdScanStatus: Self = Self(2);
    pub const TypeRespScanStatus: Self = Self(3);
    pub const TypeCmdScanResult: Self = Self(4);
    pub const TypeRespScanResult: Self = Self(5);
}
impl core::default::Default for WiFiScanMsgType {
    fn default() -> Self {
        Self(0)
    }
}
impl core::convert::From<i32> for WiFiScanMsgType {
    fn from(val: i32) -> Self {
        Self(val)
    }
}
impl core::fmt::Debug for WiFiScanMsgType {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            WiFiScanMsgType::TypeCmdScanStart => formatter.write_str("TypeCmdScanStart"),
            WiFiScanMsgType::TypeRespScanStart => formatter.write_str("TypeRespScanStart"),
            WiFiScanMsgType::TypeCmdScanStatus => formatter.write_str("TypeCmdScanStatus"),
            WiFiScanMsgType::TypeRespScanStatus => formatter.write_str("TypeRespScanStatus"),
            WiFiScanMsgType::TypeCmdScanResult => formatter.write_str("TypeCmdScanResult"),
            WiFiScanMsgType::TypeRespScanResult => formatter.write_str("TypeRespScanResult"),
            Self(n) => formatter.debug_tuple("_Unknown").field(&n).finish(),
        }
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct CmdCtrlReset {}
impl CmdCtrlReset {}
impl ::micropb::MessageDecode for CmdCtrlReset {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for CmdCtrlReset {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = ::core::result::Result::Ok(0);
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        _encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        Ok(())
    }
    fn compute_size(&self) -> usize {
        0
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct RespCtrlReset {}
impl RespCtrlReset {}
impl ::micropb::MessageDecode for RespCtrlReset {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for RespCtrlReset {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = ::core::result::Result::Ok(0);
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        _encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        Ok(())
    }
    fn compute_size(&self) -> usize {
        0
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct CmdCtrlReprov {}
impl CmdCtrlReprov {}
impl ::micropb::MessageDecode for CmdCtrlReprov {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for CmdCtrlReprov {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = ::core::result::Result::Ok(0);
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        _encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        Ok(())
    }
    fn compute_size(&self) -> usize {
        0
    }
}
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct RespCtrlReprov {}
impl RespCtrlReprov {}
impl ::micropb::MessageDecode for RespCtrlReprov {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for RespCtrlReprov {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = ::core::result::Result::Ok(0);
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        _encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        Ok(())
    }
    fn compute_size(&self) -> usize {
        0
    }
}
#[derive(Debug, Default, PartialEq, Clone)]
pub struct WiFiCtrlPayload {
    pub r#msg: WiFiCtrlMsgType,
    pub r#status: Status,
    pub r#payload: ::core::option::Option<WiFiCtrlPayload_::Payload>,
}
impl WiFiCtrlPayload {
    #[inline]
    pub fn r#msg(&self) -> &WiFiCtrlMsgType {
        &self.r#msg
    }
    #[inline]
    pub fn r#status(&self) -> &Status {
        &self.r#status
    }
}
impl ::micropb::MessageDecode for WiFiCtrlPayload {
    fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
        &mut self,
        decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
        len: usize,
    ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
        let before = decoder.bytes_read();
        while decoder.bytes_read() - before < len {
            let tag = decoder.decode_tag()?;
            match tag.field_num() {
                0 => return Err(::micropb::DecodeError::ZeroField),
                1u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let val = decoder.decode_int32().map(|n| WiFiCtrlMsgType(n as _))?;
                    if val.0 != 0 {
                        self.r#msg = val;
                    }
                }
                2u32 => {
                    if tag.wire_type() != 0u8 {
                        return Err(::micropb::DecodeError::WrongWireType);
                    }
                    let val = decoder.decode_int32().map(|n| Status(n as _))?;
                    if val.0 != 0 {
                        self.r#status = val;
                    }
                }
                11u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(
                            WiFiCtrlPayload_::Payload::CmdCtrlReset(variant),
                        ) = &mut self.r#payload
                        {
                            break variant;
                        }
                        self.r#payload =
                            ::core::option::Option::Some(WiFiCtrlPayload_::Payload::CmdCtrlReset(
                                ::core::default::Default::default(),
                            ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                12u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(
                            WiFiCtrlPayload_::Payload::RespCtrlReset(variant),
                        ) = &mut self.r#payload
                        {
                            break variant;
                        }
                        self.r#payload =
                            ::core::option::Option::Some(WiFiCtrlPayload_::Payload::RespCtrlReset(
                                ::core::default::Default::default(),
                            ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                13u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(
                            WiFiCtrlPayload_::Payload::CmdCtrlReprov(variant),
                        ) = &mut self.r#payload
                        {
                            break variant;
                        }
                        self.r#payload =
                            ::core::option::Option::Some(WiFiCtrlPayload_::Payload::CmdCtrlReprov(
                                ::core::default::Default::default(),
                            ));
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                14u32 => {
                    let mut_ref = loop {
                        if let ::core::option::Option::Some(
                            WiFiCtrlPayload_::Payload::RespCtrlReprov(variant),
                        ) = &mut self.r#payload
                        {
                            break variant;
                        }
                        self.r#payload = ::core::option::Option::Some(
                            WiFiCtrlPayload_::Payload::RespCtrlReprov(
                                ::core::default::Default::default(),
                            ),
                        );
                    };
                    decoder.decode_submessage_delimited(mut_ref)?;
                }
                _ => {
                    decoder.skip_wire_value(tag.wire_type())?;
                }
            }
        }
        Ok(())
    }
}
impl ::micropb::MessageEncode for WiFiCtrlPayload {
    const MAX_SIZE: ::core::result::Result<usize, &'static str> = ::core::result::Result::Ok(
        (WiFiCtrlMsgType::_MAX_SIZE + 1usize) + (Status::_MAX_SIZE + 1usize) + 2usize,
    );
    fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
        &self,
        encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
    ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
        if self.r#msg.0 != 0 {
            encoder.encode_varint32(8u32)?;
            encoder.encode_int32(self.r#msg.0 as _)?;
        }
        if self.r#status.0 != 0 {
            encoder.encode_varint32(16u32)?;
            encoder.encode_int32(self.r#status.0 as _)?;
        }
        if let Some(oneof) = &self.r#payload {
            match oneof {
                WiFiCtrlPayload_::Payload::CmdCtrlReset(val_ref) => {
                    encoder.encode_varint32(90u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiCtrlPayload_::Payload::RespCtrlReset(val_ref) => {
                    encoder.encode_varint32(98u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiCtrlPayload_::Payload::CmdCtrlReprov(val_ref) => {
                    encoder.encode_varint32(106u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
                WiFiCtrlPayload_::Payload::RespCtrlReprov(val_ref) => {
                    encoder.encode_varint32(114u32)?;
                    val_ref.encode_len_delimited(encoder)?;
                }
            }
        }
        Ok(())
    }
    fn compute_size(&self) -> usize {
        let mut size = 0;
        if self.r#msg.0 != 0 {
            size += 1usize + ::micropb::size::sizeof_int32(self.r#msg.0 as _);
        }
        if self.r#status.0 != 0 {
            size += 1usize + ::micropb::size::sizeof_int32(self.r#status.0 as _);
        }
        if let Some(oneof) = &self.r#payload {
            match oneof {
                WiFiCtrlPayload_::Payload::CmdCtrlReset(val_ref) => {
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiCtrlPayload_::Payload::RespCtrlReset(val_ref) => {
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiCtrlPayload_::Payload::CmdCtrlReprov(val_ref) => {
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
                WiFiCtrlPayload_::Payload::RespCtrlReprov(val_ref) => {
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.compute_size());
                }
            }
        }
        size
    }
}
/// Inner types for `WiFiCtrlPayload`
pub mod WiFiCtrlPayload_ {
    #[derive(Debug, PartialEq, Clone, Copy)]
    pub enum Payload {
        CmdCtrlReset(super::CmdCtrlReset),
        RespCtrlReset(super::RespCtrlReset),
        CmdCtrlReprov(super::CmdCtrlReprov),
        RespCtrlReprov(super::RespCtrlReprov),
    }
}
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct WiFiCtrlMsgType(pub i32);
impl WiFiCtrlMsgType {
    /// Maximum encoded size of the enum
    pub const _MAX_SIZE: usize = 10usize;
    pub const TypeCtrlReserved: Self = Self(0);
    pub const TypeCmdCtrlReset: Self = Self(1);
    pub const TypeRespCtrlReset: Self = Self(2);
    pub const TypeCmdCtrlReprov: Self = Self(3);
    pub const TypeRespCtrlReprov: Self = Self(4);
}
impl core::default::Default for WiFiCtrlMsgType {
    fn default() -> Self {
        Self(0)
    }
}
impl core::convert::From<i32> for WiFiCtrlMsgType {
    fn from(val: i32) -> Self {
        Self(val)
    }
}
impl core::fmt::Debug for WiFiCtrlMsgType {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            WiFiCtrlMsgType::TypeCtrlReserved => formatter.write_str("TypeCtrlReserved"),
            WiFiCtrlMsgType::TypeCmdCtrlReset => formatter.write_str("TypeCmdCtrlReset"),
            WiFiCtrlMsgType::TypeRespCtrlReset => formatter.write_str("TypeRespCtrlReset"),
            WiFiCtrlMsgType::TypeCmdCtrlReprov => formatter.write_str("TypeCmdCtrlReprov"),
            WiFiCtrlMsgType::TypeRespCtrlReprov => formatter.write_str("TypeRespCtrlReprov"),
            Self(n) => formatter.debug_tuple("_Unknown").field(&n).finish(),
        }
    }
}
