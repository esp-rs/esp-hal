#[derive(Debug, Clone, Copy)]
pub enum Selector {
    Set,
    Reset,
    Any,
}

impl Selector {
    pub fn into_value(&self) -> u8 {
        match self {
            Selector::Set => 0b1,
            Selector::Reset => 0b0,
            Selector::Any => 0b0,
        }
    }
    pub fn into_mask(&self) -> u8 {
        // Set bits in the mask mean we don't care about the value of that bit. The bit could be any value.
        // https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.6
        match self {
            Selector::Set => 0b0,
            Selector::Reset => 0b0,
            Selector::Any => 0b1,
        }
    }
}

impl Into<Selector> for bool {
    fn into(self) -> Selector {
        match self {
            true => Selector::Set,
            false => Selector::Reset,
        }
    }
}

pub trait BitSelectorNewExact<T, const N: usize> {
    fn new_exact(value: T) -> BitSelector<N>;
}

#[derive(Debug, Clone, Copy)]
pub struct BitSelector<const N: usize> {
    pub bits: [Selector; N],
}

impl<const N: usize> BitSelector<N> {
    pub fn new_any() -> Self {
        Self {
            bits: [Selector::Any; N],
        }
    }
}
// TODO: improve this.
impl BitSelectorNewExact<u16, 11> for BitSelector<11> {
    fn new_exact(value: u16) -> BitSelector<11> {
        Self {
            bits: [
                (value & 0b00000000001 != 0).into(),
                (value & 0b00000000010 != 0).into(),
                (value & 0b00000000100 != 0).into(),
                (value & 0b00000001000 != 0).into(),
                (value & 0b00000010000 != 0).into(),
                (value & 0b00000100000 != 0).into(),
                (value & 0b00001000000 != 0).into(),
                (value & 0b00010000000 != 0).into(),
                (value & 0b00100000000 != 0).into(),
                (value & 0b01000000000 != 0).into(),
                (value & 0b10000000000 != 0).into(),
            ],
        }
    }
}
impl BitSelectorNewExact<u8, 8> for BitSelector<8> {
    fn new_exact(value: u8) -> BitSelector<8> {
        Self {
            bits: [
                (value & 0b00000000001 != 0).into(),
                (value & 0b00000000010 != 0).into(),
                (value & 0b00000000100 != 0).into(),
                (value & 0b00000001000 != 0).into(),
                (value & 0b00000010000 != 0).into(),
                (value & 0b00000100000 != 0).into(),
                (value & 0b00001000000 != 0).into(),
                (value & 0b00010000000 != 0).into(),
            ],
        }
    }
}
impl BitSelectorNewExact<u8, 1> for BitSelector<1> {
    fn new_exact(value: u8) -> BitSelector<1> {
        Self {
            bits: [(value & 0b1 != 0).into()],
        }
    }
}
