//! Accessors for the control and status registers of CPU core.
//!
//! Covers the CSRs that light-sleep CPU retention saves and restores (see
//! [`crate::rtc_cntl::cpu_retention`]), so that the retention code can name
//! registers instead of numbers. The `riscv` crate models some of them; the
//! ones it does not know about are defined here, with the addresses ESP-IDF's
//! `csr.h` and the TRM's "RISC-V CPU" chapter use.

/// Defines CSR accessor modules shaped like the ones in `riscv::register`: a
/// safe `read() -> usize` and an `unsafe write(usize)`, so that callers can
/// treat CSRs from either crate the same way.
///
/// There are three forms. The bare `name = address` form defines a CSR the
/// `riscv` crate does not model, using that crate's accessor macros. The
/// `bits:` and `typed:` forms wrap CSRs it does model, but as a typed value
/// that callers dealing in raw register contents cannot use directly; `typed:`
/// is for the ones whose write side is typed as well.
macro_rules! define_csrs {
    (bits: $(
        $(#[$meta:meta])*
        $name:ident
    ),+ $(,)?) => {
        $(
            $(#[$meta])*
            #[allow(dead_code)]
            pub(crate) mod $name {
                #[inline(always)]
                pub(crate) fn read() -> usize {
                    riscv::register::$name::read().bits
                }

                #[inline(always)]
                pub(crate) unsafe fn write(bits: usize) {
                    unsafe { riscv::register::$name::write(bits) }
                }
            }
        )+
    };

    (typed: $(
        $(#[$meta:meta])*
        $name:ident as $ty:ident
    ),+ $(,)?) => {
        $(
            $(#[$meta])*
            #[allow(dead_code)]
            pub(crate) mod $name {
                #[inline(always)]
                pub(crate) fn read() -> usize {
                    riscv::register::$name::read().bits()
                }

                #[inline(always)]
                pub(crate) unsafe fn write(bits: usize) {
                    let value = riscv::register::$name::$ty::from_bits(bits);
                    unsafe { riscv::register::$name::write(value) }
                }
            }
        )+
    };

    ($(
        $(#[$meta:meta])*
        $name:ident = $addr:literal
    ),+ $(,)?) => {
        $(
            $(#[$meta])*
            #[allow(dead_code)]
            pub(crate) mod $name {
                riscv::read_csr_as_usize!($addr);
                riscv::write_csr_as_usize!($addr);
            }
        )+
    };
}

// CSRs whose `riscv` accessors already deal in raw bits.
pub(crate) use riscv::register::{
    mscratch,
    pmpaddr0,
    pmpaddr1,
    pmpaddr2,
    pmpaddr3,
    pmpaddr4,
    pmpaddr5,
    pmpaddr6,
    pmpaddr7,
    pmpaddr8,
    pmpaddr9,
    pmpaddr10,
    pmpaddr11,
    pmpaddr12,
    pmpaddr13,
    pmpaddr14,
    pmpaddr15,
};

define_csrs! {
    typed:
    /// Machine interrupt delegation.
    mideleg as Mideleg,
}

define_csrs! {
    bits:
    /// Physical memory protection configuration, entries 0..=3.
    pmpcfg0,
    /// Physical memory protection configuration, entries 4..=7.
    pmpcfg1,
    /// Physical memory protection configuration, entries 8..=11.
    pmpcfg2,
    /// Physical memory protection configuration, entries 12..=15.
    pmpcfg3,
}

define_csrs! {
    /// ISA and supported extensions. The `riscv` crate models this read-only;
    /// ESP-IDF writes it back after CPU power-down, so retention needs the
    /// write side as well.
    misa = 0x301,

    // Debug trigger module.
    tselect = 0x7A0,
    tdata1 = 0x7A1,
    tdata2 = 0x7A2,
    tcontrol = 0x7A5,

    // Physical memory attributes.
    pmaaddr0 = 0xBD0,
    pmaaddr1 = 0xBD1,
    pmaaddr2 = 0xBD2,
    pmaaddr3 = 0xBD3,
    pmaaddr4 = 0xBD4,
    pmaaddr5 = 0xBD5,
    pmaaddr6 = 0xBD6,
    pmaaddr7 = 0xBD7,
    pmaaddr8 = 0xBD8,
    pmaaddr9 = 0xBD9,
    pmaaddr10 = 0xBDA,
    pmaaddr11 = 0xBDB,
    pmaaddr12 = 0xBDC,
    pmaaddr13 = 0xBDD,
    pmaaddr14 = 0xBDE,
    pmaaddr15 = 0xBDF,
    pmacfg0 = 0xBC0,
    pmacfg1 = 0xBC1,
    pmacfg2 = 0xBC2,
    pmacfg3 = 0xBC3,
    pmacfg4 = 0xBC4,
    pmacfg5 = 0xBC5,
    pmacfg6 = 0xBC6,
    pmacfg7 = 0xBC7,
    pmacfg8 = 0xBC8,
    pmacfg9 = 0xBC9,
    pmacfg10 = 0xBCA,
    pmacfg11 = 0xBCB,
    pmacfg12 = 0xBCC,
    pmacfg13 = 0xBCD,
    pmacfg14 = 0xBCE,
    pmacfg15 = 0xBCF,

    // User-mode trap handling.
    ustatus = 0x000,
    utvec = 0x005,
    uepc = 0x041,
    ucause = 0x042,

    // Machine-mode performance counters.
    mpcer = 0x7E0,
    mpcmr = 0x7E1,
    mpccr = 0x7E2,
    cpu_testbus_ctrl = 0x7E3,

    // User-mode performance counters.
    upcer = 0x800,
    upcmr = 0x801,
    upccr = 0x802,

    // Dedicated GPIO.
    ugpio_oen = 0x803,
    ugpio_in = 0x804,
    ugpio_out = 0x805,
}
