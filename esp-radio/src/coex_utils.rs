#![macro_use]
#![allow(unused_macros)]

macro_rules! coex_fns {
    (
        $(
            $(#[$meta:meta])?
            fn $name:ident $args:tt $(-> $ret:ty)?;
        )*
    ) => {
        $(
            #[cfg(feature = "coex")]
            $(#[$meta])?
            pub(crate) use crate::sys::include::$name;

            #[cfg(not(feature = "coex"))]
            $(#[$meta])?
            #[allow(unused_variables)]
            pub(crate) unsafe extern "C" fn $name $args $(-> $ret)? {
                $(<$ret>::default())?
            }
        )*
    };
}

macro_rules! extern_coex_fns {
    (
        $(
            $(#[$meta:meta])?
            fn $name:ident ($($arg:ident: $arg_ty:ty),*) $(-> $ret:ty)?;
        )*
    ) => {
        $(
            $(#[$meta])?
            #[allow(unused_variables)]
            pub(crate) unsafe extern "C" fn $name ($($arg: $arg_ty),*) $(-> $ret)? {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "coex")] {
                        unsafe extern "C" {
                            fn $name($($arg: $arg_ty),*) $(-> $ret)?;
                        }
                        unsafe { $name ($($arg),*) }
                    } else {
                        $(<$ret>::default())?
                    }
                }
            }
        )*
    };
}
