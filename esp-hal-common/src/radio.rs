//! Wireless communication peripheral implementations (TBF)
//!
//! ## Overview
//! The Wireless Communication Peripheral Implementations module provides
//! implementations for different wireless communication peripherals, including
//! WiFi, Bluetooth and IEEE 802.15.4 Low Rate wireless personal area radio.
//!
//! In addition to the structures defined in this module, the module also
//! defines the `RadioExt` trait, which provides a `split` method. This method
//! allows splitting the general `Radio` peripheral into its individual
//! components.
//!
//! Additionally, the module includes implementation blocks for each wireless
//! communication peripheral, providing necessary functions and traits for each
//! peripheral.
pub trait RadioExt {
    type Components;

    fn split(self) -> Self::Components;
}

/// WiFi radio
pub struct Wifi {
    _private: (),
}

/// Bluetooth radio
pub struct Bluetooth {
    _private: (),
}

/// IEEE 802.15.4 Low rate wireless personal area radio
pub struct LowRate {
    _private: (),
}

impl Wifi {
    pub const unsafe fn steal() -> Self {
        Self { _private: () }
    }
}

impl crate::peripheral::Peripheral for Wifi {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self::steal()
    }
}

impl crate::peripheral::sealed::Sealed for Wifi {}

impl Bluetooth {
    pub const unsafe fn steal() -> Self {
        Self { _private: () }
    }
}

impl crate::peripheral::Peripheral for Bluetooth {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self::steal()
    }
}

impl crate::peripheral::sealed::Sealed for Bluetooth {}

impl LowRate {
    pub const unsafe fn steal() -> Self {
        Self { _private: () }
    }
}

impl crate::peripheral::Peripheral for LowRate {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self::steal()
    }
}

impl crate::peripheral::sealed::Sealed for LowRate {}

cfg_if::cfg_if! {
    if #[cfg(all(bt, ieee802154, wifi))] {
        impl RadioExt for crate::peripherals::RADIO {
            type Components = (Wifi, Bluetooth, LowRate);

            fn split(self) -> Self::Components {
                unsafe {
                    (Wifi::steal(), Bluetooth::steal(), LowRate::steal())
                }
            }
        }
    } else if #[cfg(all(bt, ieee802154))] {
        impl RadioExt for crate::peripherals::RADIO {
            type Components = (Bluetooth, LowRate);

            fn split(self) -> Self::Components {
                unsafe {
                    (Bluetooth::steal(), LowRate::steal())
                }
            }
        }
    } else if #[cfg(all(bt, wifi))] {
        impl RadioExt for crate::peripherals::RADIO {
            type Components = (Wifi, Bluetooth);

            fn split(self) -> Self::Components {
                unsafe {
                    (Wifi::steal(), Bluetooth::steal())
                }
            }
        }
    } else if #[cfg(wifi)] {
        impl RadioExt for crate::peripherals::RADIO {
            type Components = Wifi;

            fn split(self) -> Self::Components {
                unsafe {
                    Wifi::steal()
                }
            }
        }
    }
}
