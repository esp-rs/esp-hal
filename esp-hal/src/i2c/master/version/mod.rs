cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        mod v1;
        mod v1_v2_common;
        pub(crate) use v1::*;
    } else if #[cfg(esp32s2)] {
        mod v2;
        mod v1_v2_common;
        mod v2_v3_common;
        pub(crate) use v2::*;
    } else {
        mod v3;
        mod v2_v3_common;
        pub(crate) use v3::*;
    }
}
