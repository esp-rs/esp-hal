#![cfg_attr(docsrs, procmacros::doc_replace(
    "interface_mac" => {
        cfg(soc_has_wifi) => "let mac = efuse::interface_mac_address(InterfaceMacAddress::Station);",
        _ => "let mac = efuse::interface_mac_address(InterfaceMacAddress::Bluetooth);"
    }
))]
//! # eFuse (one-time programmable configuration)
//!
//! ## Overview
//!
//! The `efuse` module provides functionality for reading eFuse data
//! from the chip, allowing access to various chip-specific
//! information such as:
//!
//!   * MAC address
//!   * Chip revision
//!
//! and more. It is useful for retrieving chip-specific configuration and
//! identification data during runtime.
//!
//! ## Examples
//!
//! ### Reading interface MAC addresses
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::efuse::{self, InterfaceMacAddress};
//!
//! # {interface_mac}
//! println!("MAC: {mac}");
//! println!("MAC bytes: {:02x?}", mac.as_bytes());
//! # {after_snippet}
//! ```

use core::{cmp, mem, slice, sync::atomic::Ordering};

use bytemuck::AnyBitPattern;
use portable_atomic::AtomicU8;

#[cfg_attr(esp32, path = "esp32/mod.rs")]
#[cfg_attr(esp32c2, path = "esp32c2/mod.rs")]
#[cfg_attr(esp32c3, path = "esp32c3/mod.rs")]
#[cfg_attr(esp32c5, path = "esp32c5/mod.rs")]
#[cfg_attr(esp32c6, path = "esp32c6/mod.rs")]
#[cfg_attr(esp32c61, path = "esp32c61/mod.rs")]
#[cfg_attr(esp32h2, path = "esp32h2/mod.rs")]
#[cfg_attr(esp32s2, path = "esp32s2/mod.rs")]
#[cfg_attr(esp32s3, path = "esp32s3/mod.rs")]
pub(crate) mod implem;

#[instability::unstable]
pub use implem::*;
use procmacros::doc_replace;

/// The bit field for get access to efuse data
#[derive(Debug, Clone, Copy)]
#[instability::unstable]
pub struct EfuseField {
    /// The block
    pub(crate) block: EfuseBlock,
    /// Word number - this is just informational
    pub(crate) _word: u32,
    /// Starting bit in the efuse block
    pub(crate) bit_start: u32,
    /// Number of bits
    pub(crate) bit_count: u32,
}

impl EfuseField {
    pub(crate) const fn new(block: u32, word: u32, bit_start: u32, bit_count: u32) -> Self {
        Self {
            block: EfuseBlock::from_repr(block).unwrap(),
            _word: word,
            bit_start,
            bit_count,
        }
    }
}

/// Read field value in a little-endian order
#[inline(always)]
#[instability::unstable]
pub fn read_field_le<T: AnyBitPattern>(field: EfuseField) -> T {
    let EfuseField {
        block,
        bit_start,
        bit_count,
        ..
    } = field;

    // Represent output value as a bytes slice:
    let mut output = mem::MaybeUninit::<T>::uninit();
    let mut bytes =
        unsafe { slice::from_raw_parts_mut(output.as_mut_ptr() as *mut u8, mem::size_of::<T>()) };

    let bit_off = bit_start as usize;
    let bit_end = cmp::min(bit_count as usize, bytes.len() * 8) + bit_off;

    let mut last_word_off = bit_off / 32;
    let mut last_word = unsafe { block.address().add(last_word_off).read_volatile() };

    let word_bit_off = bit_off % 32;
    let word_bit_ext = 32 - word_bit_off;

    let mut word_off = last_word_off;
    for bit_off in (bit_off..bit_end).step_by(32) {
        if word_off != last_word_off {
            // Read a new word:
            last_word_off = word_off;
            last_word = unsafe { block.address().add(last_word_off).read_volatile() };
        }

        let mut word = last_word >> word_bit_off;
        word_off += 1;

        let word_bit_len = cmp::min(bit_end - bit_off, 32);
        if word_bit_len > word_bit_ext {
            // Read the next word:
            last_word_off = word_off;
            last_word = unsafe { block.address().add(last_word_off).read_volatile() };
            // Append bits from a beginning of the next word:
            word |= last_word.wrapping_shl((32 - word_bit_off) as u32);
        };

        if word_bit_len < 32 {
            // Mask only needed bits of a word:
            word &= u32::MAX >> (32 - word_bit_len);
        }

        // Represent word as a byte slice:
        let byte_len = word_bit_len.div_ceil(8);
        let word_bytes =
            unsafe { slice::from_raw_parts(&word as *const u32 as *const u8, byte_len) };

        // Copy word bytes to output value bytes:
        bytes[..byte_len].copy_from_slice(word_bytes);

        // Move read window forward:
        bytes = &mut bytes[byte_len..];
    }

    // Fill untouched bytes with zeros:
    bytes.fill(0);

    unsafe { output.assume_init() }
}

/// Read bit value.
///
/// This function panics if the field's bit length is not equal to 1.
#[inline(always)]
#[instability::unstable]
pub fn read_bit(field: EfuseField) -> bool {
    assert_eq!(field.bit_count, 1);
    read_field_le::<u8>(field) != 0
}

/// Overrides the base MAC address used by [`interface_mac_address`].
///
/// After a successful call, [`interface_mac_address`] will derive
/// per-interface addresses from the overridden base instead of the factory
/// eFuse MAC. [`base_mac_address`] is unaffected and continues to return the
/// factory eFuse value.
///
/// The override does not persist across device resets.
/// Can only be called once. Returns `Err(SetMacError::AlreadySet)`
/// otherwise.
#[instability::unstable]
pub fn override_mac_address(mac: MacAddress) -> Result<(), SetMacError> {
    if MAC_OVERRIDE_STATE
        .compare_exchange(0, 1, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        return Err(SetMacError::AlreadySet);
    }

    unsafe {
        MAC_OVERRIDE = mac;
    }

    MAC_OVERRIDE_STATE.store(2, Ordering::Release);

    Ok(())
}

#[procmacros::doc_replace]
/// Returns the base MAC address programmed into eFuse during manufacturing.
///
/// This always reads directly from the hardware eFuse storage. To get the
/// effective MAC for a specific radio interface (which may be overridden via
/// [`override_mac_address`]), use [`interface_mac_address`] instead.
///
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::efuse;
///
/// let mac = efuse::base_mac_address();
/// println!("Base MAC: {mac}");
/// # {after_snippet}
/// ```
#[instability::unstable]
pub fn base_mac_address() -> MacAddress {
    let mut mac_addr = [0u8; 6];

    let mac0 = read_field_le::<[u8; 4]>(crate::efuse::MAC0);
    let mac1 = read_field_le::<[u8; 2]>(crate::efuse::MAC1);

    // MAC address is stored in big endian, so load the bytes in reverse:
    mac_addr[0] = mac1[1];
    mac_addr[1] = mac1[0];
    mac_addr[2] = mac0[3];
    mac_addr[3] = mac0[2];
    mac_addr[4] = mac0[1];
    mac_addr[5] = mac0[0];

    MacAddress::new_eui48(mac_addr)
}

#[procmacros::doc_replace(
    "interface_mac_example" => {
        cfg(soc_has_wifi) => "let mac = efuse::interface_mac_address(InterfaceMacAddress::Station);",
        _ => "let mac = efuse::interface_mac_address(InterfaceMacAddress::Bluetooth);"
    }
)]
/// Returns the MAC address for a specific interface, derived from the base
/// MAC.
///
/// By default, addresses are derived from the factory eFuse MAC returned
/// by [`base_mac_address`]. If [`override_mac_address`] has been called, the
/// overridden base address is used instead.
///
/// Each chip is programmed with a unique base MAC address during manufacturing.
/// Different interfaces (Wi-Fi Station, SoftAP, Bluetooth, etc.) each use a
/// MAC address derived from this base address. The Station interface uses
/// the base MAC directly, while others use locally administered variants
/// produced by modifying the first octet to set the local-admin bit; some
/// interfaces (such as Bluetooth) additionally adjust the last octet to
/// obtain a distinct address.
///
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::efuse::{self, InterfaceMacAddress};
///
/// # {interface_mac_example}
/// println!("MAC: {mac}");
/// # {after_snippet}
/// ```
#[cfg(any(soc_has_wifi, soc_has_bt))]
pub fn interface_mac_address(kind: InterfaceMacAddress) -> MacAddress {
    let mut mac = if MAC_OVERRIDE_STATE.load(Ordering::Acquire) == 2 {
        unsafe { MAC_OVERRIDE }
    } else {
        base_mac_address()
    };

    match kind {
        #[cfg(soc_has_wifi)]
        InterfaceMacAddress::Station => {
            // base MAC
        }
        #[cfg(soc_has_wifi)]
        InterfaceMacAddress::AccessPoint => {
            derive_local_mac(&mut mac);
        }
        #[cfg(soc_has_bt)]
        InterfaceMacAddress::Bluetooth => {
            derive_local_mac(&mut mac);

            mac.0[5] = mac.0[5].wrapping_add(1);
        }
    }
    mac
}

#[doc_replace]
/// Returns the hardware revision.
///
/// ## Examples
///
/// ```rust,no_run
/// # {before_snippet}
/// let rev = esp_hal::efuse::chip_revision();
/// println!("Chip revision: {}.{}", rev.major, rev.minor);
/// # {after_snippet}
/// ```
#[inline]
pub fn chip_revision() -> ChipRevision {
    ChipRevision {
        major: major_chip_version(),
        minor: minor_chip_version(),
    }
}

#[doc_replace]
/// Represents the hardware revision.
///
/// The type supports converting between two separate u16-based representations:
///
/// - Combined: a `u16` calculated as `major * 100 + minor`. The combined representation is more
///   often used by ESP-IDF, and working with it involves integer division. Note that the combined
///   representation assumes minor is less than 100.
/// - Packed: a `u16` with the major revision in the high byte and the minor revision in the low
///   byte.
///
/// ## Examples
///
/// ```rust,no_run
/// # {before_snippet}
/// use esp_hal::efuse::ChipRevision;
///
/// let rev = esp_hal::efuse::chip_revision();
/// println!("Chip revision: {}.{}", rev.major, rev.minor);
///
/// // You can compare against other ChipRevision objects
/// if rev < ChipRevision::from_combined(300) {
///     // You can print a debug representation
///     println!("Chip revision is too old: {:?}", rev);
/// }
/// if rev >= ChipRevision::from_packed(0x400) {
///     println!("Chip revision is too new: {:?}", rev);
/// }
///
/// // You can convert into two different numeric representations.
/// assert!(rev.packed() >= 0x300);
/// assert!(rev.combined() >= 300);
/// # {after_snippet}
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChipRevision {
    /// The major revision number.
    pub major: u8,

    /// The minor revision number.
    pub minor: u8,
}

impl PartialOrd for ChipRevision {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ChipRevision {
    #[inline]
    fn cmp(&self, other: &Self) -> cmp::Ordering {
        match self.major.cmp(&other.major) {
            cmp::Ordering::Equal => self.minor.cmp(&other.minor),
            ord => ord,
        }
    }
}

impl ChipRevision {
    /// Creates a new [`ChipRevision`] from a combined revision value.
    #[inline]
    pub const fn from_combined(revision: u16) -> Self {
        let major = revision / 100;
        let minor = revision % 100;
        ::core::assert!(
            major <= u8::MAX as u16 && minor <= u8::MAX as u16,
            "`ChipRevision` cannot represent revision",
        );
        Self {
            major: major as u8,
            minor: minor as u8,
        }
    }

    /// Returns the combined revision value as a `u16`.
    ///
    /// The combined revision value is a `u16` calculated as `major * 100 + minor`.
    #[inline]
    pub const fn combined(self) -> u16 {
        ::core::assert!(
            self.minor < 100,
            "`ChipRevision` cannot be represented using the combined representation",
        );
        (self.major as u16) * 100 + (self.minor as u16)
    }

    /// Creates a new [`ChipRevision`] from a packed revision value.
    ///
    /// The packed revision value is a `u16` with the major revision in the high byte and the minor
    /// revision in the low byte.
    #[inline]
    pub const fn from_packed(packed: u16) -> Self {
        Self {
            major: ((packed >> 8) & 0xFF) as u8,
            minor: (packed & 0xFF) as u8,
        }
    }

    /// Returns the packed revision value as a `u16`.
    ///
    /// The packed revision value is a `u16` with the major revision in the high byte and the minor
    /// revision in the low byte.
    #[inline]
    pub const fn packed(self) -> u16 {
        (self.major as u16) << 8 | (self.minor as u16)
    }
}

// Indicates the state of setting the mac address
// 0 -- unset
// 1 -- in the process of being set
// 2 -- set
//
// Values other than 0 indicate that we cannot attempt setting the mac address
// again, and values other than 2 indicate that we should read the mac address
// from eFuse.
static MAC_OVERRIDE_STATE: AtomicU8 = AtomicU8::new(0);
static mut MAC_OVERRIDE: MacAddress = MacAddress::new_eui48([0; 6]);

/// Error indicating issues with setting the MAC address.
#[derive(PartialEq, Eq, Hash, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum SetMacError {
    /// The MAC address has already been set and cannot be changed.
    AlreadySet,
}

impl core::fmt::Display for SetMacError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SetMacError::AlreadySet => write!(f, "The MAC address has already been set"),
        }
    }
}

impl core::error::Error for SetMacError {}

/// Helper function.
/// Serves to derive a local MAC by adjusting the first octet of the given base MAC.
/// See https://github.com/esp-rs/esp-hal/blob/0881d747c53e43ee847bef3068076a48ce8d27f0/esp-radio/src/common_adapter.rs#L151-L159
#[cfg(any(soc_has_wifi, soc_has_bt))]
fn derive_local_mac(mac: &mut MacAddress) {
    let bytes = &mut mac.0;
    let base = bytes[0];

    for i in 0..64 {
        let derived = (base | 0x02) ^ (i << 2);
        if derived != base {
            bytes[0] = derived;
            break;
        }
    }
}

/// Interface selection for [`interface_mac_address`].
///
/// Each interface uses a distinct MAC address derived from either the base MAC or the overridden
/// MAC if [`override_mac_address`] has been called.
#[derive(PartialEq, Eq, Hash, Copy, Clone, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(any(soc_has_wifi, soc_has_bt))]
#[non_exhaustive]
pub enum InterfaceMacAddress {
    /// Wi-Fi station. Equivalent to the base MAC address or overridden via
    /// [`override_mac_address`].
    #[cfg(soc_has_wifi)]
    #[cfg_attr(soc_has_wifi, default)]
    Station,
    /// Wi-Fi SoftAP.
    #[cfg(soc_has_wifi)]
    AccessPoint,
    /// Bluetooth (BT/BLE).
    #[cfg(soc_has_bt)]
    #[cfg_attr(not(soc_has_wifi), default)]
    Bluetooth,
}

/// Hardware (MAC) address.
///
/// Use [`as_bytes`](Self::as_bytes) for raw access, or the
/// [`Display`](core::fmt::Display) impl for colon-separated hex
/// (e.g. `aa:bb:cc:dd:ee:ff`).
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MacAddress([u8; 6]);

impl MacAddress {
    /// Creates a new `MacAddress` from the given bytes.
    #[instability::unstable]
    pub const fn new_eui48(bytes: [u8; 6]) -> Self {
        Self(bytes)
    }

    /// Returns the address bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }
}

impl core::fmt::Display for MacAddress {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for (i, b) in self.0.iter().enumerate() {
            if i != 0 {
                f.write_str(":")?;
            }
            write!(f, "{b:02x}")?;
        }
        Ok(())
    }
}
