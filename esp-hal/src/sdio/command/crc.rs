//! CRC7 checksum algorithm for command checksums.
//!
//! Authored-by: sdmmc-core developers
//! Originally licensed as GPLv3
//! Permitted for distribution under APACHE or MIT by esp-rs

const CRC7_POLY: u8 = 0b1000_1001;
const CRC7_MASK: u8 = 0x7f;

/// Represents the 7-bit CRC used to protect SD memory card commands, responses, and data transfer
/// messages.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Crc(u8);

impl Crc {
    /// Creates a new [Crc].
    pub const fn new() -> Self {
        Self(0)
    }

    /// Gets the inner representation of the [Crc] bits.
    pub const fn into_u8(self) -> u8 {
        self.0
    }

    /// Converts a [`u8`] into a [Crc].
    pub const fn from_u8(val: u8) -> Self {
        Self(val & CRC7_MASK)
    }

    /// Calculates the CRC7 value according the algorithm defined in the simplified physical
    /// specification. See section "4.5 Cyclic Redundancy Code" for details.
    ///
    /// ```no_build,no_run
    /// Generator Polynomial: G(x) = x^7 + x^3 + 1
    /// M(x) = (first bit) * x^n + (second bit) * x^n-1 + ... + (last bit) * x^0
    /// CRC[6..0] = Remainder[(M(x) * x^7) / G(x)]
    /// ```
    ///
    /// Implementation based on the lookup table algorithm from: [hazelnusse/crc7](https://github.com/hazelnusse/crc7).
    pub const fn calculate(data: &[u8]) -> Self {
        let mut crc = 0;
        let mut i = 0;
        let len = data.len();

        while i < len {
            crc = Self::crc_table((crc << 1) ^ data[i]);
            i = i.saturating_add(1);
        }

        Self(crc)
    }

    // Calculates the CRC-7 lookup value based on the `crc` value.
    #[inline(always)]
    const fn crc_table(mut crc: u8) -> u8 {
        crc ^= Self::crc_rem(crc);
        let mut j = 1;

        while j < 8 {
            crc = (crc << 1) ^ Self::crc_rem(crc << 1);
            j += 1;
        }

        crc
    }

    // Used to clear leading bit from CRC value.
    //
    // If the leading bit is set, adds the CRC-7 polynomial to correct the value.
    #[inline(always)]
    const fn crc_rem(val: u8) -> u8 {
        if val & 0x80 != 0 { CRC7_POLY } else { 0 }
    }
}

impl Default for Crc {
    fn default() -> Self {
        Self::new()
    }
}

impl From<u8> for Crc {
    fn from(val: u8) -> Self {
        Self::from_u8(val)
    }
}

impl From<Crc> for u8 {
    fn from(val: Crc) -> Self {
        val.into_u8()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc7() {
        [0b1001010, 0b0101010, 0b0110011]
            .map(Crc::from)
            .into_iter()
            .zip([
                [0b0100_0000, 0x00, 0x00, 0x00, 0x00],
                [0b0101_0001, 0x00, 0x00, 0x00, 0x00],
                [0b0001_0001, 0x00, 0x00, 0b0000_1001, 0x00],
            ])
            .for_each(|(exp_crc, data)| {
                assert_eq!(Crc::calculate(data.as_ref()), exp_crc);
            });
    }
}
