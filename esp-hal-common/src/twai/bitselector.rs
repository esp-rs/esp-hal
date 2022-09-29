use core::ops::Shr;

/// A single bit of a bitmask filter.
///
/// A Set bit will match only a set bit, a Reset bit will match only a reset bit, and Any
/// will match any bit value.
///
/// The EXACT const generic determines the interpretation of the mask bit. A true value means
/// that set bits in the mask match the value exactly. A false value means that reset
/// bits in the mask match exactly.
///
#[derive(Debug, Clone, Copy)]
pub enum Selector<const EXACT: bool> {
    Set,
    Reset,
    Any,
}

impl<const EXACT: bool> From<u8> for Selector<EXACT> {
    fn from(value: u8) -> Self {
        if value & 0b1 != 0 {
            Selector::Set
        } else {
            Selector::Reset
        }
    }
}
impl<const EXACT: bool> From<u16> for Selector<EXACT> {
    fn from(value: u16) -> Self {
        (value as u8).into()
    }
}
impl<const EXACT: bool> From<u32> for Selector<EXACT> {
    fn from(value: u32) -> Self {
        (value as u8).into()
    }
}
impl<const EXACT: bool> From<u64> for Selector<EXACT> {
    fn from(value: u64) -> Self {
        (value as u8).into()
    }
}

/// A bitmask filter.
#[derive(Debug, Clone, Copy)]
pub struct BitSelector<const EXACT: bool, const N: usize> {
    pub bits: [Selector<EXACT>; N],
}

impl<const EXACT: bool, const N: usize> BitSelector<EXACT, N> {
    pub fn new(bits: [Selector<EXACT>; N]) -> Self {
        Self { bits }
    }
    /// Create a new selector that matches any value.
    pub fn new_any() -> Self {
        Self {
            bits: [Selector::Any; N],
        }
    }
}

/// A trait to convert a value into a selector of a specified size that exactly matches the value.
///
/// This will convert type T into a selector of size N bits. The EXACT is forwarded to the
/// selector and it's meaning is further explained in the selector docs.
pub trait BitSelectorNewExact<T, const EXACT: bool, const N: usize>
where
    T: Into<Selector<EXACT>> + Shr<u8, Output = T> + Copy,
{
    /// Create a new selector that matches exactly the given value.
    fn new_exact(mut value: T) -> BitSelector<EXACT, N> {
        let mut bits = [Selector::Any; N];

        for bit in bits.iter_mut() {
            *bit = value.into();
            value = value >> 1;
        }

        BitSelector { bits }
    }
}

impl<const EXACT: bool> BitSelectorNewExact<u16, EXACT, 11> for BitSelector<EXACT, 11> {}
impl<const EXACT: bool> BitSelectorNewExact<u8, EXACT, 8> for BitSelector<EXACT, 8> {}
impl<const EXACT: bool> BitSelectorNewExact<u8, EXACT, 1> for BitSelector<EXACT, 1> {}

/// Trait to convert selector type into values and masks.
pub trait SelectorInto<T> {
    /// Convert the selector into the value portion of the filter.
    fn to_value(&self) -> T;
    /// Convert the selector into the mask portion of the filter.
    fn to_mask(&self) -> T;
}

impl<const EXACT: bool> SelectorInto<u8> for Selector<EXACT> {
    fn to_value(&self) -> u8 {
        match self {
            Selector::Set => 0b1,
            Selector::Reset => 0b0,
            Selector::Any => 0b0,
        }
    }
    fn to_mask(&self) -> u8 {
        match self {
            Selector::Set => EXACT as u8,
            Selector::Reset => EXACT as u8,
            Selector::Any => (!EXACT) as u8,
        }
    }
}

impl<const EXACT: bool> SelectorInto<u8> for [Selector<EXACT>] {
    fn to_value(&self) -> u8 {
        let mut value: u8 = 0;

        for bit in self.iter().rev() {
            let v: u8 = bit.to_value();
            value = (value << 1) | v;
        }

        value
    }
    fn to_mask(&self) -> u8 {
        let mut mask: u8 = 0;

        // Fill the unused portion of the mask with bits that match against anything.
        for _ in 0..8 {
            mask = (mask << 1) | ((!EXACT) as u8);
        }

        // Insert the bits from the slice into the mask.
        for bit in self.iter().rev() {
            let m: u8 = bit.to_mask();
            mask = (mask << 1) | m;
        }

        mask
    }
}

impl<const EXACT: bool> SelectorInto<u8> for BitSelector<EXACT, 8> {
    fn to_value(&self) -> u8 {
        self.bits.as_slice().to_value()
    }
    fn to_mask(&self) -> u8 {
        self.bits.as_slice().to_mask()
    }
}
impl<const EXACT: bool> core::fmt::Display for Selector<EXACT> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let b_str = match self {
            Selector::Any => "x",
            Selector::Reset => "0",
            Selector::Set => "1",
        };
        write!(f, "{}", b_str)
    }
}

impl<const EXACT: bool, const N: usize> core::fmt::Display for BitSelector<EXACT, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "0b")?;
        for bit in self.bits.iter().rev() {
            write!(f, "{}", bit)?;
        }
        Ok(())
    }
}
