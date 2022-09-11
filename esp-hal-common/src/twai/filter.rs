use super::bitselector::BitSelector;

#[derive(Debug, PartialEq)]
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

///
///
/// Warning: This is not a perfect filter. Extended ids that match the bit layout of this filter
/// will also be accepted.
pub struct SingleStandardFilter {
    pub id: BitSelector<11>,
    pub rtr: BitSelector<1>,
    pub data: [BitSelector<8>; 2],
}

impl Filter for SingleStandardFilter {
    const FILTER_TYPE: FilterType = FilterType::Single;
    fn to_registers(&self) -> [u8; 8] {
        [
            // Value.
            // TODO: Implement some sort of into for slices of bits so that we can simplify some of this code.
            self.id.bits[10].into_value() << 7
                | self.id.bits[9].into_value() << 6
                | self.id.bits[8].into_value() << 5
                | self.id.bits[7].into_value() << 4
                | self.id.bits[6].into_value() << 3
                | self.id.bits[5].into_value() << 2
                | self.id.bits[4].into_value() << 1
                | self.id.bits[3].into_value(),
            self.id.bits[2].into_value() << 7
                | self.id.bits[1].into_value() << 6
                | self.id.bits[0].into_value() << 5
                | self.rtr.bits[0].into_value() << 4,
            self.data[0].bits[7].into_value() << 7
                | self.data[0].bits[6].into_value() << 6
                | self.data[0].bits[5].into_value() << 5
                | self.data[0].bits[4].into_value() << 4
                | self.data[0].bits[3].into_value() << 3
                | self.data[0].bits[2].into_value() << 2
                | self.data[0].bits[1].into_value() << 1
                | self.data[0].bits[0].into_value() << 0,
            self.data[1].bits[7].into_value() << 7
                | self.data[1].bits[6].into_value() << 6
                | self.data[1].bits[5].into_value() << 5
                | self.data[1].bits[4].into_value() << 4
                | self.data[1].bits[3].into_value() << 3
                | self.data[1].bits[2].into_value() << 2
                | self.data[1].bits[1].into_value() << 1
                | self.data[1].bits[0].into_value() << 0,
            // Mask.
            self.id.bits[10].into_mask() << 7
                | self.id.bits[9].into_mask() << 6
                | self.id.bits[8].into_mask() << 5
                | self.id.bits[7].into_mask() << 4
                | self.id.bits[6].into_mask() << 3
                | self.id.bits[5].into_mask() << 2
                | self.id.bits[4].into_mask() << 1
                | self.id.bits[3].into_mask(),
            self.id.bits[2].into_mask() << 7
                | self.id.bits[1].into_mask() << 6
                | self.id.bits[0].into_mask() << 5
                | self.rtr.bits[0].into_mask() << 4,
            self.data[0].bits[7].into_mask() << 7
                | self.data[0].bits[6].into_mask() << 6
                | self.data[0].bits[5].into_mask() << 5
                | self.data[0].bits[4].into_mask() << 4
                | self.data[0].bits[3].into_mask() << 3
                | self.data[0].bits[2].into_mask() << 2
                | self.data[0].bits[1].into_mask() << 1
                | self.data[0].bits[0].into_mask() << 0,
            self.data[1].bits[7].into_mask() << 7
                | self.data[1].bits[6].into_mask() << 6
                | self.data[1].bits[5].into_mask() << 5
                | self.data[1].bits[4].into_mask() << 4
                | self.data[1].bits[3].into_mask() << 3
                | self.data[1].bits[2].into_mask() << 2
                | self.data[1].bits[1].into_mask() << 1
                | self.data[1].bits[0].into_mask() << 0,
        ]
    }
}
///
///
/// Warning: This is not a perfect filter. Standard ids that match the bit layout of this filter
/// will also be accepted.
pub struct SingleExtendedFilter {
    pub id: BitSelector<29>,
    pub rtr: BitSelector<1>,
}
impl Filter for SingleExtendedFilter {
    const FILTER_TYPE: FilterType = FilterType::Single;
    fn to_registers(&self) -> [u8; 8] {
        panic!("Unimplemented");
    }
}

///
/// TODO: is this how we actually want to store the two filters?
///
/// Warning: This is not a perfect filter. Extended ids that match the bit layout of this filter
/// will also be accepted.
pub struct DualStandardFilter {
    pub first_id: BitSelector<11>,
    pub first_rtr: BitSelector<1>,
    pub first_data: BitSelector<8>,

    pub second_id: BitSelector<11>,
    pub second_rtr: BitSelector<1>,
}
impl Filter for DualStandardFilter {
    const FILTER_TYPE: FilterType = FilterType::Dual;
    fn to_registers(&self) -> [u8; 8] {
        // TODO: this.
        panic!("Unimplemented");
    }
}
///
/// NOTE: The dual extended id acceptance filters can only match "the first 16 bits of the 29-bit ID".
///
///
/// Warning: This is not a perfect filter. Standard ids that match the bit layout of this filter
/// will also be accepted.
pub struct DualExtendedFilter {
    pub id: [BitSelector<16>; 2],
}
impl Filter for DualExtendedFilter {
    const FILTER_TYPE: FilterType = FilterType::Dual;
    fn to_registers(&self) -> [u8; 8] {
        // TODO: this.
        panic!("Unimplemented");
    }
}
