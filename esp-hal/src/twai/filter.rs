//! Two-wire Automotive Interface (TWAI) Filters
//!
//! ## Overview
//!
//! The TWAI controller contains a hardware acceptance filter which can be used
//! to filter messages of a particular ID. A node that filters out a message
//! does not receive the message, but will still acknowledge it. Acceptance
//! filters can make a node more efficient by filtering out messages sent over
//! the bus that are irrelevant to the node.
//!
//! ## Configuration
//!
//! The acceptance filters are configured using two 32-bit values known as the
//! acceptance code and the acceptance mask.

use super::{ExtendedId, StandardId};

#[derive(Debug, PartialEq, Eq)]
/// Represents the type of filtering to be applied to incoming TWAI frames.
pub enum FilterType {
    /// Uses the acceptance code and mask to define a single filter, which
    /// allows for the first two data bytes of a standard frame to be filtered,
    /// or the entirety of an extended frame's 29-bit ID.
    Single,
    /// Uses the acceptance code and mask to define two separate filters
    /// allowing for increased flexibility of ID's to accept, but does not allow
    /// for all 29-bits of an extended ID to be filtered.
    Dual,
}

/// Interface for interacting with Acceptance Filters.
///
/// The Acceptance Filter is a programmable message filtering unit that allows
/// the TWAI controller to accept or reject a received message based on the
/// message’s ID field.
///
/// Only accepted messages will be stored in the Receive FIFO.
///
/// The Acceptance Filter’s registers can be programmed to specify a single
/// filter, or two separate filters (dual filter mode).
pub trait Filter {
    /// The type of the filter.
    const FILTER_TYPE: FilterType;
    /// Returns filter type.
    fn filter_type(&self) -> FilterType {
        Self::FILTER_TYPE
    }

    /// Get the register level representation of the filter.
    fn to_registers(&self) -> [u8; 8];
}

/// A type representing the bitmask used to filter incoming TWAI frames.
pub type BitFilter<const N: usize> = [u8; N];

// Convert a byte from a bytestring into a bit inside a given code and mask.
macro_rules! set_bit_from_byte {
    ($code:expr_2021, $mask:expr_2021, $byte:expr_2021, $shift:expr_2021) => {
        match $byte {
            b'0' => {
                // Code bit is already zero, no need to set it.
                $mask |= 1 << $shift;
            }
            b'1' => {
                $code |= 1 << $shift;
                $mask |= 1 << $shift;
            }
            b'x' => {}
            _ => ::core::panic!("BitFilter bits must be either '1', '0' or 'x'."),
        }
    };
}

// Convert a code and mask to the byte array needed at a register level.
//
// On the input mask, set bits (1) mean we care about the exact value of the
// corresponding bit in the code, reset bits (0) mean the bit could be any
// value.
const fn code_mask_to_register_array(code: u32, mask: u32) -> [u8; 8] {
    // Convert the filter code and mask into the full byte array needed for the
    // registers.
    let [code_3, code_2, code_1, code_0] = code.to_be_bytes();

    // At a register level, set bits in the mask mean we don't care about the value
    // of that bit. Therefore, we invert the mask.
    // https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.6
    let [mask_3, mask_2, mask_1, mask_0] = (!mask).to_be_bytes();

    [
        code_3, code_2, code_1, code_0, mask_3, mask_2, mask_1, mask_0,
    ]
}

/// A filter that matches against a single 11 bit id, the RTR bit, and the first
/// two bytes of the payload.
///
/// Warning: This is not a perfect filter. Extended IDs that match the bit
/// layout of this filter will also be accepted.
pub struct SingleStandardFilter {
    /// The register representation of the filter.
    raw: [u8; 8],
}

impl SingleStandardFilter {
    /// Create a new filter that matches against a single 11-bit standard id.
    /// The filter can match against the packet's id, RTR bit, and first two
    /// bytes of the payload.
    ///
    /// Example matching only even IDs, allowing any rtr value and any payload
    /// data:
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::twai::filter::SingleStandardFilter;
    /// const FILTER: SingleStandardFilter =
    ///     SingleStandardFilter::new(
    ///         b"xxxxxxxxxx0",
    ///         b"x",
    ///         [b"xxxxxxxx", b"xxxxxxxx"]
    ///     );
    /// # Ok(())
    /// # }
    /// ```
    pub const fn new(id: &BitFilter<11>, rtr: &BitFilter<1>, payload: [&BitFilter<8>; 2]) -> Self {
        // The bit values we desire to match against. This determines whether we want a
        // set bit (1) or a reset bit (0).
        let mut acceptance_code: u32 = 0;
        // The acceptance mask, set bits (1) mean we care about the exact value of the
        // corresponding bit in the code, reset bits (0) mean the bit could be any
        // value.
        let mut acceptance_mask: u32 = 0;

        // Convert the id filter into the code and mask bits.
        {
            let mut idx = 0;
            while idx < 11 {
                let shift = 31 - idx;
                set_bit_from_byte!(acceptance_code, acceptance_mask, id[idx], shift);
                idx += 1;
            }
        }
        // Convert the RTR bit filter into the code and mask bits.
        {
            let shift = 20;
            set_bit_from_byte!(acceptance_code, acceptance_mask, rtr[0], shift);
        }
        // Convert the payload byte filter into the code and mask bits.
        {
            let mut payload_index = 0;
            while payload_index < 2 {
                let mut idx = 0;
                while idx < 8 {
                    let shift = 15 - (8 * payload_index) - idx;
                    set_bit_from_byte!(
                        acceptance_code,
                        acceptance_mask,
                        payload[payload_index][idx],
                        shift
                    );
                    idx += 1;
                }
                payload_index += 1;
            }
        }

        Self {
            raw: code_mask_to_register_array(acceptance_code, acceptance_mask),
        }
    }

    /// The masks indicate which bits of the code the filter should match
    /// against. Set bits in the mask indicate that the corresponding bit in
    /// the code should match.
    ///
    ///
    /// # Examples
    ///
    /// A filter that matches every standard id that is even, is not an rtr
    /// frame, with any bytes for the first two payload bytes.
    /// ```rust, ignore
    /// let filter = twai::filter::SingleStandardFilter::new_from_code_mask(
    ///     StandardId::new(0x000)?,
    ///     StandardId::new(0x001)?,
    ///     false,
    ///     true,
    ///     [0x00, 0x00],
    ///     [0x00, 0x00],
    /// );
    /// ```
    pub fn new_from_code_mask(
        id_code: StandardId,
        id_mask: StandardId,
        rtr_code: bool,
        rtr_mask: bool,
        payload_code: [u8; 2],
        payload_mask: [u8; 2],
    ) -> Self {
        // The bit values we desire to match against. This determines whether we want a
        // set bit (1) or a reset bit (0).
        let mut acceptance_code: u32 = 0;
        // The acceptance mask, set bits (1) mean we care about the exact value of the
        // corresponding bit in the code, reset bits (0) mean the bit could be any
        // value.
        let mut acceptance_mask: u32 = 0;

        // Pack the id into the full layout.
        acceptance_code |= (id_code.as_raw() as u32) << 21;
        acceptance_mask |= (id_mask.as_raw() as u32) << 21;

        // Pack the RTR bit into the full layout.
        acceptance_code |= (rtr_code as u32) << 20;
        acceptance_mask |= (rtr_mask as u32) << 20;

        // Pack the payload bytes into the full layout.
        acceptance_code |= ((payload_code[0] as u32) << 8) | (payload_code[1] as u32);
        acceptance_mask |= ((payload_mask[0] as u32) << 8) | (payload_mask[1] as u32);

        Self {
            raw: code_mask_to_register_array(acceptance_code, acceptance_mask),
        }
    }
}

impl Filter for SingleStandardFilter {
    const FILTER_TYPE: FilterType = FilterType::Single;
    fn to_registers(&self) -> [u8; 8] {
        self.raw
    }
}

/// Warning: This is not a perfect filter. Standard IDs that match the bit
/// layout of this filter will also be accepted.
pub struct SingleExtendedFilter {
    raw: [u8; 8],
}

impl SingleExtendedFilter {
    /// Create a filter that matches against a single 29-bit extended id.
    ///
    /// The filter can match against the packet's id and the RTR bit.
    ///
    /// # Examples
    /// A filter matching any odd extended IDs, with any rtr value.
    /// ```rust, ignore
    /// const FILTER: twai::filter::SingleExtendedFilter =
    ///     twai::filter::SingleExtendedFilter::new(b"xxxxxxxxxxxxxxxxxxxxxxxxxxxx1", b"x");
    /// ```
    pub const fn new(id: &BitFilter<29>, rtr: &BitFilter<1>) -> Self {
        // The bit values we desire to match against. This determines whether we want a
        // set bit (1) or a reset bit (0).
        let mut acceptance_code: u32 = 0;
        // The acceptance mask, set bits (1) mean we care about the exact value of the
        // corresponding bit in the code, reset bits (0) mean the bit could be any
        // value.
        let mut acceptance_mask: u32 = 0;

        // Convert the id filter into the code and mask bits.
        {
            let mut idx = 0;
            while idx < 29 {
                let shift = 31 - idx;
                set_bit_from_byte!(acceptance_code, acceptance_mask, id[idx], shift);
                idx += 1;
            }
        }
        // Convert the RTR bit filter into the code and mask bits.
        {
            let shift = 2;
            set_bit_from_byte!(acceptance_code, acceptance_mask, rtr[0], shift);
        }

        Self {
            raw: code_mask_to_register_array(acceptance_code, acceptance_mask),
        }
    }
    /// The masks indicate which bits of the code the filter should match
    /// against. Set bits in the mask indicate that the corresponding bit in
    /// the code should match.
    pub fn new_from_code_mask(
        id_code: ExtendedId,
        id_mask: ExtendedId,
        rtr_code: bool,
        rtr_mask: bool,
    ) -> Self {
        // The bit values we desire to match against. This determines whether we want a
        // set bit (1) or a reset bit (0).
        let mut acceptance_code: u32 = 0;
        // The acceptance mask, set bits (1) mean we care about the exact value of the
        // corresponding bit in the code, reset bits (0) mean the bit could be any
        // value.
        let mut acceptance_mask: u32 = 0;

        // Pack the id into the full layout.
        acceptance_code |= id_code.as_raw() << 3;
        acceptance_mask |= id_mask.as_raw() << 3;

        // Pack the RTR bit into the full layout.
        acceptance_code |= (rtr_code as u32) << 2;
        acceptance_mask |= (rtr_mask as u32) << 2;

        Self {
            raw: code_mask_to_register_array(acceptance_code, acceptance_mask),
        }
    }
}

impl Filter for SingleExtendedFilter {
    const FILTER_TYPE: FilterType = FilterType::Single;
    fn to_registers(&self) -> [u8; 8] {
        self.raw
    }
}

/// A filter that matches against two standard 11-bit standard IDs.
///
/// The first filter part can match a packet's id, RTR bit, and the first byte
/// of the payload. The second filter part can match a packet's id and RTR bit.
///
/// Warning: This is not a perfect filter. Extended IDs that match the bit
/// layout of this filter will also be accepted.
pub struct DualStandardFilter {
    raw: [u8; 8],
}

impl DualStandardFilter {
    /// Create a filter that matches against two standard 11-bit standard IDs.
    ///
    /// The first filter part can match a packet's id, RTR bit, and the first
    /// byte of the payload. The second filter part can match a packet's id
    /// and RTR bit.
    ///
    /// # Examples
    /// A filter that matches any standard id that ends with a 00 or a 11, with
    /// any RTR, and with any payload on the first filter.
    /// ```rust, ignore
    /// const FILTER: twai::filter::DualStandardFilter = twai::filter::DualStandardFilter::new(
    ///     b"xxxxxxxxx00",
    ///     b"x",
    ///     b"xxxxxxxx",
    ///     b"xxxxxxxxx11",
    ///     b"x",
    /// );
    /// ```
    pub const fn new(
        first_id: &BitFilter<11>,
        first_rtr: &BitFilter<1>,
        first_payload: &BitFilter<8>,
        second_id: &BitFilter<11>,
        second_rtr: &BitFilter<1>,
    ) -> Self {
        // The bit values we desire to match against. This determines whether we want a
        // set bit (1) or a reset bit (0).
        let mut acceptance_code: u32 = 0;
        // The acceptance mask, set bits (1) mean we care about the exact value of the
        // corresponding bit in the code, reset bits (0) mean the bit could be any
        // value.
        let mut acceptance_mask: u32 = 0;

        // Convert the first id filter into the code and mask bits.
        {
            let mut idx = 0;
            while idx < 11 {
                let shift = 31 - idx;
                set_bit_from_byte!(acceptance_code, acceptance_mask, first_id[idx], shift);
                idx += 1;
            }
        }
        // Convert the first RTR bit filter into the code and mask bits.
        {
            let shift = 20;
            set_bit_from_byte!(acceptance_code, acceptance_mask, first_rtr[0], shift);
        }
        // Convert the first payload byte filter into the code and mask bits.
        {
            let mut idx = 0;
            while idx < 4 {
                let shift = 19 - idx;
                set_bit_from_byte!(acceptance_code, acceptance_mask, first_payload[idx], shift);
                idx += 1;
            }
            while idx < 8 {
                let shift = 3 + 4 - idx;
                set_bit_from_byte!(acceptance_code, acceptance_mask, first_payload[idx], shift);
                idx += 1;
            }
        }
        // Convert the second id filter into the code and mask bits.
        {
            let mut idx = 0;
            while idx < 11 {
                let shift = 15 - idx;
                set_bit_from_byte!(acceptance_code, acceptance_mask, second_id[idx], shift);
                idx += 1;
            }
        }
        // Convert the second RTR bit filter into the code and mask bits.
        {
            let shift = 4;
            set_bit_from_byte!(acceptance_code, acceptance_mask, second_rtr[0], shift);
        }

        Self {
            raw: code_mask_to_register_array(acceptance_code, acceptance_mask),
        }
    }
    /// The masks indicate which bits of the code the filter should match
    /// against. Set bits in the mask indicate that the corresponding bit in
    /// the code should match.
    #[allow(clippy::too_many_arguments)]
    pub fn new_from_code_mask(
        first_id_code: StandardId,
        first_id_mask: StandardId,
        first_rtr_code: bool,
        first_rtr_mask: bool,
        first_payload_code: u8,
        first_payload_mask: u8,
        second_id_code: StandardId,
        second_id_mask: StandardId,
        second_rtr_code: bool,
        second_rtr_mask: bool,
    ) -> Self {
        // The bit values we desire to match against. This determines whether we want a
        // set bit (1) or a reset bit (0).
        let mut acceptance_code: u32 = 0;
        // The acceptance mask, set bits (1) mean we care about the exact value of the
        // corresponding bit in the code, reset bits (0) mean the bit could be any
        // value.
        let mut acceptance_mask: u32 = 0;

        // Pack the first id into the full layout.
        acceptance_code |= (first_id_code.as_raw() as u32) << 21;
        acceptance_mask |= (first_id_mask.as_raw() as u32) << 21;

        // Pack the RTR bit into the full layout.
        acceptance_code |= (first_rtr_code as u32) << 20;
        acceptance_mask |= (first_rtr_mask as u32) << 20;

        // Pack the first payload into the full layout.
        acceptance_code |= ((first_payload_code & 0xF0) as u32) << 12;
        acceptance_mask |= ((first_payload_mask & 0xF0) as u32) << 12;
        acceptance_code |= (first_payload_code & 0x0F) as u32;
        acceptance_mask |= (first_payload_mask & 0x0F) as u32;

        // Pack the second id into the full layout.
        acceptance_code |= (second_id_code.as_raw() as u32) << 5;
        acceptance_mask |= (second_id_mask.as_raw() as u32) << 5;

        // Pack the second RTR bit into the full layout.
        acceptance_code |= (second_rtr_code as u32) << 4;
        acceptance_mask |= (second_rtr_mask as u32) << 4;

        Self {
            raw: code_mask_to_register_array(acceptance_code, acceptance_mask),
        }
    }
}

impl Filter for DualStandardFilter {
    const FILTER_TYPE: FilterType = FilterType::Dual;
    fn to_registers(&self) -> [u8; 8] {
        self.raw
    }
}

/// Warning: This is not a perfect filter. Standard IDs that match the bit
/// layout of this filter will also be accepted.
///
/// NOTE: The dual extended id acceptance filters can only match "the first 16
/// bits of the 29-bit ID".
pub struct DualExtendedFilter {
    raw: [u8; 8],
}

impl DualExtendedFilter {
    /// Create a filter that matches the first 16 bits of two 29-bit extended
    /// IDs.
    ///
    /// # Examples
    /// A filter that matches IDs with 4 bits either set or reset in the higher
    /// part of the id. For example this id matches: 0x000f000f, 0x000f000a,
    /// 0x0000000a, 0x0000000b.
    /// But it does not match: 0x000a000a
    /// ```rust, ignore
    /// const FILTER: twai::filter::DualExtendedFilter =
    ///     twai::filter::DualExtendedFilter::new([b"xxxxxxxxx0000xxx", b"xxxxxxxxx1111xxx"]);
    /// ```
    pub const fn new(ids: [&BitFilter<16>; 2]) -> Self {
        // The bit values we desire to match against. This determines whether we want a
        // set bit (1) or a reset bit (0).
        let mut acceptance_code: u32 = 0;
        // The acceptance mask, set bits (1) mean we care about the exact value of the
        // corresponding bit in the code, reset bits (0) mean the bit could be any
        // value.
        let mut acceptance_mask: u32 = 0;

        // Convert the id filters into the code and mask bits.
        {
            let mut filter_idx = 0;
            while filter_idx < 2 {
                let mut idx = 0;
                while idx < 16 {
                    let shift = 31 - (filter_idx * 16) - idx;
                    set_bit_from_byte!(
                        acceptance_code,
                        acceptance_mask,
                        ids[filter_idx][idx],
                        shift
                    );
                    idx += 1;
                }
                filter_idx += 1;
            }
        }

        Self {
            raw: code_mask_to_register_array(acceptance_code, acceptance_mask),
        }
    }
    /// Create a new filter matching the first 16 bits of two 29-bit IDs.
    ///
    /// The masks indicate which bits of the code the filter should match
    /// against. Set bits in the mask indicate that the corresponding bit in
    /// the code should match.
    pub fn new_from_code_mask(ids_code: [u16; 2], ids_mask: [u16; 2]) -> Self {
        // The bit values we desire to match against. This determines whether we want a
        // set bit (1) or a reset bit (0).
        let mut acceptance_code: u32 = 0;
        // The acceptance mask, set bits (1) mean we care about the exact value of the
        // corresponding bit in the code, reset bits (0) mean the bit could be any
        // value.
        let mut acceptance_mask: u32 = 0;

        // Pack the first partial id into the full layout.
        acceptance_code |= (ids_code[0] as u32) << 16;
        acceptance_mask |= (ids_mask[0] as u32) << 16;

        // Pack the second partial id into the full layout.
        acceptance_code |= ids_code[1] as u32;
        acceptance_mask |= ids_mask[1] as u32;

        Self {
            raw: code_mask_to_register_array(acceptance_code, acceptance_mask),
        }
    }
}

impl Filter for DualExtendedFilter {
    const FILTER_TYPE: FilterType = FilterType::Dual;
    fn to_registers(&self) -> [u8; 8] {
        self.raw
    }
}
