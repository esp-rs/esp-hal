use embedded_hal::can::{ExtendedId, StandardId};

pub struct ValueMask<T> {
    pub value: T,
    pub mask: T,
}
pub trait FilterToRegisters {
    fn to_registers(self) -> [u8; 8];
}

pub struct SingleStandardFilter {
    pub id: ValueMask<StandardId>,
    pub rtr: ValueMask<bool>,
    pub data: ValueMask<[u8; 2]>,
}
impl FilterToRegisters for SingleStandardFilter {
    fn to_registers(self) -> [u8; 8] {
        [
            // Value.
            (self.id.value.as_raw() >> 3) as u8,
            ((self.id.value.as_raw() << 5) as u8
                | if self.rtr.value { 0b1 << 4 } else { 0b0 << 4 })
                & 0b11110000,
            self.data.value[0],
            self.data.value[1],
            // Mask.
            (self.id.mask.as_raw() >> 3) as u8,
            ((self.id.mask.as_raw() << 5) as u8 | if self.rtr.mask { 0b1 << 4 } else { 0b0 << 4 })
                & 0b11110000,
            self.data.mask[0],
            self.data.mask[1],
        ]
    }
}

pub struct SingleExtendedFilter {
    pub id: ValueMask<ExtendedId>,
    pub rtr: ValueMask<bool>,
}
impl FilterToRegisters for SingleExtendedFilter {
    fn to_registers(self) -> [u8; 8] {
        [
            // Value.
            (self.id.value.as_raw() >> 21) as u8,
            (self.id.value.as_raw() >> 13) as u8,
            (self.id.value.as_raw() >> 5) as u8,
            ((self.id.value.as_raw() << 3) as u8
                | if self.rtr.value { 0b1 << 2 } else { 0b0 << 2 })
                & 0b11111100,
            // Mask.
            (self.id.mask.as_raw() >> 21) as u8,
            (self.id.mask.as_raw() >> 13) as u8,
            (self.id.mask.as_raw() >> 5) as u8,
            ((self.id.mask.as_raw() << 3) as u8 | if self.rtr.mask { 0b1 << 2 } else { 0b0 << 2 })
                & 0b11111100,
        ]
    }
}

// TODO: how do we actually want to store the two filters?

pub struct DualStandardFilter {
    pub id: ValueMask<StandardId>,
    pub rtr: ValueMask<bool>,
    // TODO: only the first filter can match on the data.
    pub data: ValueMask<[u8; 1]>,
}
impl FilterToRegisters for DualStandardFilter {
    fn to_registers(self) -> [u8; 8] {
        // TODO: this.
        panic!("Unimplemented");
    }
}
///
/// NOTE: The dual extended id acceptance filter can only match "the first 16 bits of the 29-bit ID".
pub struct DualExtendedFilter {
    pub id: ValueMask<u16>,
}
impl FilterToRegisters for DualExtendedFilter {
    fn to_registers(self) -> [u8; 8] {
        // TODO: this.
        panic!("Unimplemented");
    }
}

pub enum FilterIdFormat<Std, Ext> {
    Standard(Std),
    Extended(Ext),
}
impl<Std, Ext> FilterToRegisters for FilterIdFormat<Std, Ext>
where
    Std: FilterToRegisters,
    Ext: FilterToRegisters,
{
    fn to_registers(self) -> [u8; 8] {
        match self {
            FilterIdFormat::Standard(filter) => filter.to_registers(),
            FilterIdFormat::Extended(filter) => filter.to_registers(),
        }
    }
}

pub enum Filter {
    Single(FilterIdFormat<SingleStandardFilter, SingleExtendedFilter>),
    Dual(FilterIdFormat<DualStandardFilter, DualExtendedFilter>),
}

impl FilterToRegisters for Filter {
    fn to_registers(self) -> [u8; 8] {
        match self {
            Self::Single(single) => single.to_registers(),
            Self::Dual(dual) => dual.to_registers(),
        }
    }
}
