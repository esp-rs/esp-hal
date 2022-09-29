use super::bitselector::{BitSelector, SelectorInto};

#[derive(Debug, PartialEq)]
pub enum FilterType {
    Single,
    Dual,
}

// Set bits in the mask mean we don't care about the value of that bit. The bit could be any value.
// https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.6

pub trait Filter {
    // The type of the filter.
    const FILTER_TYPE: FilterType;
    fn filter_type(&self) -> FilterType {
        Self::FILTER_TYPE
    }

    fn to_registers(&self) -> [u8; 8];
}

///
///
/// Warning: This is not a perfect filter. Extended ids that match the bit layout of this filter
/// will also be accepted.
pub struct SingleStandardFilter {
    pub id: BitSelector<false, 11>,
    pub rtr: BitSelector<false, 1>,
    pub data: [BitSelector<false, 8>; 2],
}

impl Filter for SingleStandardFilter {
    const FILTER_TYPE: FilterType = FilterType::Single;
    fn to_registers(&self) -> [u8; 8] {
        [
            // Value.
            self.id.bits[3..11].to_value(),
            self.id.bits[0..3].to_value() << 5 | self.rtr.bits[0].to_value() << 4,
            self.data[0].to_value(),
            self.data[1].to_value(),
            // Mask.
            self.id.bits[3..11].to_mask(),
            self.id.bits[0..3].to_mask() << 5 | self.rtr.bits[0].to_mask() << 4,
            self.data[0].to_mask(),
            self.data[1].to_mask(),
        ]
    }
}
///
///
/// Warning: This is not a perfect filter. Standard ids that match the bit layout of this filter
/// will also be accepted.
pub struct SingleExtendedFilter {
    pub id: BitSelector<false, 29>,
    pub rtr: BitSelector<false, 1>,
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
    pub first_id: BitSelector<false, 11>,
    pub first_rtr: BitSelector<false, 1>,
    pub first_data: BitSelector<false, 8>,

    pub second_id: BitSelector<false, 11>,
    pub second_rtr: BitSelector<false, 1>,
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
    pub id: [BitSelector<false, 16>; 2],
}
impl Filter for DualExtendedFilter {
    const FILTER_TYPE: FilterType = FilterType::Dual;
    fn to_registers(&self) -> [u8; 8] {
        todo!();
    }
}
