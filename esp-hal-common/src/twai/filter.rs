#[derive(Debug, PartialEq, Eq)]
pub enum FilterType {
    Single,
    Dual,
}

pub trait Filter {
    // The type of the filter.
    const FILTER_TYPE: FilterType;
    fn filter_type(&self) -> FilterType {
        Self::FILTER_TYPE
    }

    fn to_registers(&self) -> [u8; 8];
}

pub type BitFilter<const N: usize> = [u8; N];

/// A filter that matches against a single 11 bit id, the rtr bit, and the first two bytes of the
/// payload.
///
/// Warning: This is not a perfect filter. Extended ids that match the bit layout of this filter
/// will also be accepted.
pub struct SingleStandardFilter {
    /// The register representation of the filter.
    raw: [u8; 8],
}

impl SingleStandardFilter {
    /// Create a new filter that matches against the id, rtr and first two bytes of the payload.
    ///
    /// Example matching only even ids, allowing any rtr value and any payload data:
    /// ```
    /// const FILTER: SingleStandardFilter =
    ///     SingleStandardFilter::new(b"xxxxxxxxxx0", b"x", [b"xxxxxxxx", b"xxxxxxxx"]);
    /// ```
    pub const fn new(id: &BitFilter<11>, rtr: &BitFilter<1>, payload: [&BitFilter<8>; 2]) -> Self {
        // The bit values we desire to match against. This determines whether we want a set
        // bit (1) or a reset bit (0).
        let mut acceptance_code: u32 = 0;
        // The acceptance mask, set bits (1) mean we care about the exact value of the
        // corresponding bit in the code, reset bits (0) mean the bit could be any value.
        let mut acceptance_mask: u32 = 0;

        // Convert the id filter into the code and mask bits.
        {
            let mut idx = 0;
            while idx < 11 {
                let shift = 31 - idx;

                match id[idx] {
                    b'0' => {
                        // Code bit is already zero, no need to set it.
                        acceptance_mask |= 1 << shift;
                    }
                    b'1' => {
                        acceptance_code |= 1 << shift;
                        acceptance_mask |= 1 << shift;
                    }
                    b'x' => {}
                    _ => panic!("BitFilter bits must be either '1', '0' or 'x'."),
                }

                idx += 1;
            }
        }
        // Convert the rtr bit filter into the code and mask bits.
        {
            let shift = 20;
            match rtr[0] {
                b'0' => {
                    // Code bit is already zero, no need to set it.
                    acceptance_mask |= 1 << shift;
                }
                b'1' => {
                    acceptance_code |= 1 << shift;
                    acceptance_mask |= 1 << shift;
                }
                b'x' => {}
                _ => panic!("BitFilter bits must be either '1', '0' or 'x'."),
            }
        }
        // Convert the payload byte filter into the code and mask bits.
        {
            let mut payload_index = 0;
            while payload_index < 2 {
                let mut idx = 0;
                while idx < 8 {
                    let shift = 15 - (8 * payload_index) - idx;

                    match payload[payload_index][idx] {
                        b'0' => {
                            // Code bit is already zero, no need to set it.
                            acceptance_mask |= 1 << shift;
                        }
                        b'1' => {
                            acceptance_code |= 1 << shift;
                            acceptance_mask |= 1 << shift;
                        }
                        b'x' => {}
                        _ => panic!("BitFilter bits must be either '1', '0' or 'x'."),
                    }

                    idx += 1;
                }

                payload_index += 1;
            }
        }

        // Convert the filter code and mask into the full byte array needed for the registers.
        let [code_3, code_2, code_1, code_0] = acceptance_code.to_be_bytes();

        // At a register level, set bits in the mask mean we don't care about the value of that bit. Therefore, we invert the mask.
        // https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.6
        let [mask_3, mask_2, mask_1, mask_0] = (!acceptance_mask).to_be_bytes();

        Self {
            raw: [
                code_3, code_2, code_1, code_0, mask_3, mask_2, mask_1, mask_0,
            ],
        }
    }
}

impl Filter for SingleStandardFilter {
    const FILTER_TYPE: FilterType = FilterType::Single;
    fn to_registers(&self) -> [u8; 8] {
        self.raw
    }
}
///
///
/// Warning: This is not a perfect filter. Standard ids that match the bit layout of this filter
/// will also be accepted.
pub struct SingleExtendedFilter {
    // pub id: BitSelector<false, 29>,
    // pub rtr: BitSelector<false, 1>,
}
impl Filter for SingleExtendedFilter {
    const FILTER_TYPE: FilterType = FilterType::Single;
    fn to_registers(&self) -> [u8; 8] {
        todo!();
    }
}

///
/// TODO: is this how we actually want to store the two filters?
///
/// Warning: This is not a perfect filter. Extended ids that match the bit layout of this filter
/// will also be accepted.
pub struct DualStandardFilter {
    // pub first_id: BitSelector<false, 11>,
    // pub first_rtr: BitSelector<false, 1>,
    // pub first_data: BitSelector<false, 8>,

    // pub second_id: BitSelector<false, 11>,
    // pub second_rtr: BitSelector<false, 1>,
}
impl Filter for DualStandardFilter {
    const FILTER_TYPE: FilterType = FilterType::Dual;
    fn to_registers(&self) -> [u8; 8] {
        todo!();
    }
}
///
/// NOTE: The dual extended id acceptance filters can only match "the first 16 bits of the 29-bit ID".
///
///
/// Warning: This is not a perfect filter. Standard ids that match the bit layout of this filter
/// will also be accepted.
pub struct DualExtendedFilter {
    // pub id: [BitSelector<false, 16>; 2],
}
impl Filter for DualExtendedFilter {
    const FILTER_TYPE: FilterType = FilterType::Dual;
    fn to_registers(&self) -> [u8; 8] {
        todo!();
    }
}
