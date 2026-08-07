//! CSRs saved and restored by CPU power-down retention.
//!
//! Retention round-trips whole registers as raw bits. Entries from
//! [`riscv::register`] use `read_bits()`/`write_bits()` so typed CSRs whose
//! field masks would otherwise drop bits survive
//! the round trip. Everything else is generated here with
//! [`riscv::read_write_csr_as_usize`].
//!
//! Order matches ESP-IDF `sleep_cpu.c`.

use procmacros::ram;

/// Reads one retention-list entry as raw bits.
macro_rules! retained_read {
    (riscv $name:ident) => {
        riscv::register::$name::read_bits()
    };
    (local $name:ident) => {
        $name::read()
    };
}

/// Writes raw bits back to one retention-list entry.
macro_rules! retained_write {
    (riscv $name:ident, $bits:expr) => {
        unsafe { riscv::register::$name::write_bits($bits) }
    };
    (local $name:ident, $bits:expr) => {
        unsafe { $name::write($bits) }
    };
}

/// Defines the non-critical retention list.
macro_rules! noncritical_csrs {
    ($( $kind:tt $name:ident $(= $addr:literal)? ),+ $(,)?) => {
        $($(
            #[allow(dead_code)]
            mod $name {
                riscv::read_write_csr_as_usize!($addr);
            }
        )?)+

        /// Non-critical CSR slot count; sizes the `noncritical` field.
        pub(crate) const NONCRITICAL_WORDS: usize = [$(stringify!($name)),+].len();

        #[ram]
        pub(crate) fn save_noncritical(buf: *mut u32) {
            let mut i = 0usize;
            $(
                let bits = retained_read!($kind $name) as u32;
                unsafe { buf.add(i).write(bits) };
                i += 1;
            )+
            let _ = i;
        }

        #[ram]
        pub(crate) fn restore_noncritical(buf: *const u32) {
            let mut i = 0usize;
            $(
                let bits = unsafe { buf.add(i).read() } as usize;
                retained_write!($kind $name, bits);
                i += 1;
            )+
            let _ = i;
        }
    };
}

noncritical_csrs! {
    riscv mscratch,
    riscv mideleg,
    riscv misa,

    riscv tselect,
    riscv tdata1,
    riscv tdata2,
    riscv tcontrol,

    riscv pmpaddr0,
    riscv pmpaddr1,
    riscv pmpaddr2,
    riscv pmpaddr3,
    riscv pmpaddr4,
    riscv pmpaddr5,
    riscv pmpaddr6,
    riscv pmpaddr7,
    riscv pmpaddr8,
    riscv pmpaddr9,
    riscv pmpaddr10,
    riscv pmpaddr11,
    riscv pmpaddr12,
    riscv pmpaddr13,
    riscv pmpaddr14,
    riscv pmpaddr15,

    riscv pmpcfg0,
    riscv pmpcfg1,
    riscv pmpcfg2,
    riscv pmpcfg3,

    local pmaaddr0 = 0xbd0,
    local pmaaddr1 = 0xbd1,
    local pmaaddr2 = 0xbd2,
    local pmaaddr3 = 0xbd3,
    local pmaaddr4 = 0xbd4,
    local pmaaddr5 = 0xbd5,
    local pmaaddr6 = 0xbd6,
    local pmaaddr7 = 0xbd7,
    local pmaaddr8 = 0xbd8,
    local pmaaddr9 = 0xbd9,
    local pmaaddr10 = 0xbda,
    local pmaaddr11 = 0xbdb,
    local pmaaddr12 = 0xbdc,
    local pmaaddr13 = 0xbdd,
    local pmaaddr14 = 0xbde,
    local pmaaddr15 = 0xbdf,

    local pmacfg0 = 0xbc0,
    local pmacfg1 = 0xbc1,
    local pmacfg2 = 0xbc2,
    local pmacfg3 = 0xbc3,
    local pmacfg4 = 0xbc4,
    local pmacfg5 = 0xbc5,
    local pmacfg6 = 0xbc6,
    local pmacfg7 = 0xbc7,
    local pmacfg8 = 0xbc8,
    local pmacfg9 = 0xbc9,
    local pmacfg10 = 0xbca,
    local pmacfg11 = 0xbcb,
    local pmacfg12 = 0xbcc,
    local pmacfg13 = 0xbcd,
    local pmacfg14 = 0xbce,
    local pmacfg15 = 0xbcf,

    local utvec = 0x005,
    local ustatus = 0x000,
    local uepc = 0x041,
    local ucause = 0x042,

    local mpcer = 0x7e0,
    local mpcmr = 0x7e1,
    local mpccr = 0x7e2,
    local cpu_testbus_ctrl = 0x7e3,

    local upcer = 0x800,
    local upcmr = 0x801,
    local upccr = 0x802,

    local ugpio_oen = 0x803,
    local ugpio_in = 0x804,
    local ugpio_out = 0x805,
}
