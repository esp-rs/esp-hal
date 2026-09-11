//! Retention buffer layout and device regions, per chip.

use crate::{
    rtc_cntl::cpu_retention::{
        device_regs::DeviceRegion,
        frames::chip::{CRITICAL_FRAME_SIZE, NON_CRITICAL_FRAME_SIZE},
    },
    system::Cpu,
};

/// Runtime absolute address (as `u32`) of a named PAC register.
macro_rules! reg {
    ($peri:ident, [$($path:tt)+]) => {
        unsafe { &*crate::pac::$peri::PTR }.$($path)+.as_ptr() as *const u32
    };
}

macro_rules! saved_region_map {
    ($(
        $( #[cfg($cfg:tt)] )? {
            $peri:ident, $start:tt, $count:expr
        },
    )*) => {
        const REGION_COUNT: usize = 0 $(+ {
            1 $( * cfg!($cfg) as usize )?
        })*;

        pub(crate) const DEVICE_REGION_WORDS: usize = 0 $(+ {
            ($count) $( * cfg!($cfg) as usize )?
        })*;

        #[inline(always)]
        pub(crate) fn regions() -> [DeviceRegion; REGION_COUNT] {
            [
                $(
                    $(#[cfg($cfg)])?
                    DeviceRegion::new(reg!($peri, $start), $count),
                )*
            ]
        }
    };
}

#[cfg(any(esp32h2, esp32c6))]
saved_region_map! {
    { INTPRI, [cpu_int_enable()], 45 },
    { INTPRI, [rnd_eco_high()], 1 },
    #[cfg(esp32c6)] // H2 onward this is read-only, no need to save
    { EXTMEM, [l1_cache_ctrl()], 1 },
    { EXTMEM, [l1_cache_wrap_around_ctrl()], 1 },
    { PLIC_MX, [mxint_enable()], 38 },
    { PLIC_MX, [mxint_conf()], 1 },
    { PLIC_UX, [uxint_enable()], 38 },
    { PLIC_UX, [uxint_conf()], 1 },
    { CLINT, [msip()], 6 },
    { CLINT, [usip()], 6 },
}

#[cfg(any(esp32c5, esp32c61))]
saved_region_map! {
    { CACHE, [cache_autoload_ctrl()], 5 },
    { CLIC, [int_config()], 3 },
    { CLIC, [int_ip(0)], 48 },
    { CLINT, [msip()], 1 },
    { CLINT, [mtimecmp()], 2 },
    { CLINT, [mtimectl()], 1 },
    { CLINT, [mtime()], 2 },
}

#[cfg(esp32p4)]
saved_region_map! {
    { CLIC, [int_config()], 3 },
    { CLIC, [int_ip(0)], 48 },
}

// FIXME: find out why this isn't equivalent to P4
#[cfg(esp32s31)]
saved_region_map! {
    { CLIC, [int_config()], 1 },
    { CLIC, [int_thresh()], 1 },
    { CLIC, [int_ip(0)], 48 },
}

pub(crate) const CRITICAL_FRAME_OFFSET: usize = 0;
pub(crate) const NON_CRITICAL_FRAME_OFFSET: usize = CRITICAL_FRAME_OFFSET + CRITICAL_FRAME_SIZE;
pub(crate) const DEVICE_REGIONS_OFFSET: usize = NON_CRITICAL_FRAME_OFFSET + NON_CRITICAL_FRAME_SIZE;

/// Bytes one core's frames need, rounded up to the alignment of the buffer.
pub(crate) const BLOCK_SIZE: usize =
    (DEVICE_REGIONS_OFFSET + DEVICE_REGION_WORDS * 4).next_multiple_of(16);

/// Bytes every core's frames need.
pub(crate) const BUFFER_SIZE: usize = BLOCK_SIZE * Cpu::COUNT;

/// `RTC_SLEEP_WAKE_STUB_ADDR_REG`: the word that holds the wake stub address across the sleep.
#[inline(always)]
pub(crate) fn wake_stub_reg() -> *mut u32 {
    cfg_select! {
        esp32s31 => {
            crate::peripherals::LP_SYS::regs().lp_store(8).as_ptr()
        }
        esp32p4 => {
            crate::peripherals::LP_AON::regs().lp_store8().as_ptr()
        }
        _ => {
            crate::peripherals::LP_AON::regs().store8().as_ptr()
        }
    }
}

// The metadata carries the size, because the install API checks the buffer against it before any
// frame code runs.
const _: () = ::core::assert!(BUFFER_SIZE == super::memory::BUFFER_SIZE);
