#![macro_use]
#![allow(unused_macros)]

macro_rules! assert {
    ($($x:tt)*) => {
        {
            ::core::assert!($($x)*);
        }
    };
}

macro_rules! assert_eq {
    ($($x:tt)*) => {
        {
            ::core::assert_eq!($($x)*);
        }
    };
}

macro_rules! assert_ne {
    ($($x:tt)*) => {
        {
            ::core::assert_ne!($($x)*);
        }
    };
}

macro_rules! debug_assert {
    ($($x:tt)*) => {
        {
            ::core::debug_assert!($($x)*);
        }
    };
}

macro_rules! debug_assert_eq {
    ($($x:tt)*) => {
        {
            ::core::debug_assert_eq!($($x)*);
        }
    };
}

macro_rules! debug_assert_ne {
    ($($x:tt)*) => {
        {
            ::core::debug_assert_ne!($($x)*);
        }
    };
}

macro_rules! todo {
    ($($x:tt)*) => {
        {
            ::core::todo!($($x)*);
        }
    };
}

macro_rules! unreachable {
    ($($x:tt)*) => {
        {
            ::core::unreachable!($($x)*);
        }
    };
}

macro_rules! panic {
    ($($x:tt)*) => {
        {
            ::core::panic!($($x)*);
        }
    };
}

macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "log")]
            ::log::trace!($s $(, $x)*);
            #[cfg(not(feature = "log"))]
            let _ = ($( & $x ),*);
        }
    };
}

macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "log")]
            ::log::debug!($s $(, $x)*);
            #[cfg(not(feature = "log"))]
            let _ = ($( & $x ),*);
        }
    };
}

macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "log")]
            ::log::info!($s $(, $x)*);
            #[cfg(not(feature = "log"))]
            let _ = ($( & $x ),*);
        }
    };
}

macro_rules! warn {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "log")]
            ::log::warn!($s $(, $x)*);
            #[cfg(not(feature = "log"))]
            let _ = ($( & $x ),*);
        }
    };
}

macro_rules! error {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "log")]
            ::log::error!($s $(, $x)*);
            #[cfg(not(feature = "log"))]
            let _ = ($( & $x ),*);
        }
    };
}

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
