#![macro_use]
#![allow(unused_macros)]

use core::fmt::{Debug, Display, LowerHex};

#[collapse_debuginfo(yes)]
macro_rules! assert {
    ($($x:tt)*) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::assert!($($x)*);
                } else {
                    ::core::assert!($($x)*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! assert_eq {
    ($($x:tt)*) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::assert_eq!($($x)*);
                } else {
                    ::core::assert_eq!($($x)*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! assert_ne {
    ($($x:tt)*) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::assert_ne!($($x)*);
                } else {
                    ::core::assert_ne!($($x)*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! debug_assert {
    ($($x:tt)*) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::debug_assert!($($x)*);
                } else {
                    ::core::debug_assert!($($x)*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! debug_assert_eq {
    ($($x:tt)*) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::debug_assert_eq!($($x)*);
                } else {
                    ::core::debug_assert_eq!($($x)*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! debug_assert_ne {
    ($($x:tt)*) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::debug_assert_ne!($($x)*);
                } else {
                    ::core::debug_assert_ne!($($x)*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! todo {
    ($($x:tt)*) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::todo!($($x)*);
                } else {
                    ::core::todo!($($x)*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! unreachable {
    ($($x:tt)*) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::unreachable!($($x)*);
                } else {
                    ::core::unreachable!($($x)*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! panic {
    ($($x:tt)*) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::panic!($($x)*);
                } else {
                    ::core::panic!($($x)*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::trace!($s $(, $x)*);
                } else if #[cfg(feature = "log-04")] {
                    ::log_04::trace!($s $(, $x)*);
                } else {
                    let _ = ($( & $x ),*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::debug!($s $(, $x)*);
                } else if #[cfg(feature = "log-04")] {
                    ::log_04::debug!($s $(, $x)*);
                } else {
                    let _ = ($( & $x ),*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::info!($s $(, $x)*);
                } else if #[cfg(feature = "log-04")] {
                    ::log_04::info!($s $(, $x)*);
                } else {
                    let _ = ($( & $x ),*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! warn {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::warn!($s $(, $x)*);
                } else if #[cfg(feature = "log-04")] {
                    ::log_04::warn!($s $(, $x)*);
                } else {
                    let _ = ($( & $x ),*);
                }
            }
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! error {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            cfg_if::cfg_if! {
                if #[cfg(feature = "defmt")] {
                    ::defmt::error!($s $(, $x)*);
                } else if #[cfg(feature = "log-04")] {
                    ::log_04::error!($s $(, $x)*);
                } else {
                    let _ = ($( & $x ),*);
                }
            }
        }
    };
}

#[cfg(feature = "defmt")]
#[collapse_debuginfo(yes)]
macro_rules! unwrap {
    ($($x:tt)*) => {
        ::defmt::unwrap!($($x)*)
    };
}

#[cfg(not(feature = "defmt"))]
#[collapse_debuginfo(yes)]
macro_rules! unwrap {
    ($arg:expr) => {
        match $crate::fmt::Try::into_result($arg) {
            ::core::result::Result::Ok(t) => t,
            ::core::result::Result::Err(e) => {
                ::core::panic!("unwrap of `{}` failed: {:?}", ::core::stringify!($arg), e);
            }
        }
    };
    ($arg:expr, $($msg:expr),+ $(,)? ) => {
        match $crate::fmt::Try::into_result($arg) {
            ::core::result::Result::Ok(t) => t,
            ::core::result::Result::Err(e) => {
                ::core::panic!("unwrap of `{}` failed: {}: {:?}", ::core::stringify!($arg), ::core::format_args!($($msg,)*), e);
            }
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct NoneError;

pub trait Try {
    type Ok;
    type Error;
    #[allow(unused)]
    fn into_result(self) -> Result<Self::Ok, Self::Error>;
}

impl<T> Try for Option<T> {
    type Ok = T;
    type Error = NoneError;

    #[inline]
    fn into_result(self) -> Result<T, NoneError> {
        self.ok_or(NoneError)
    }
}

impl<T, E> Try for Result<T, E> {
    type Ok = T;
    type Error = E;

    #[inline]
    fn into_result(self) -> Self {
        self
    }
}

/// A way to `{:x?}` format a byte slice which is compatible with `defmt`
pub struct Bytes<'a>(pub &'a [u8]);

impl Debug for Bytes<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:#02x?}", self.0)
    }
}

impl Display for Bytes<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:#02x?}", self.0)
    }
}

impl LowerHex for Bytes<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:#02x?}", self.0)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Bytes<'_> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{:02x}", self.0)
    }
}
