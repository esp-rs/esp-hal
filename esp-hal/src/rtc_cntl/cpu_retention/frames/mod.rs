//! Sleep frame layouts.
//!
//! The chips fall into three groups, and the groups do not share a layout. See
//! `components/esp_hw_support/lowpower/port/<chip>/rvsleep-frames.h`.

/// Declares the critical frame and the matched pair that moves it.
///
/// One list drives the struct, the save and the restore, so the three cannot drift apart.
macro_rules! critical_frame {
    (
        { // fields in the group - each inherits the cfg
            $($field:ident),* $(,)?
        }
        $(, // conditional groups
            #[$cfg:meta]
            { // fields in the group - each inherits the cfg
                $($cfg_field:ident),* $(,)?
            }
        )+
    ) => {
        /// Registers that the assembly saves, because the CPU has no valid state at that point.
        #[repr(C)]
        pub(crate) struct CriticalSleepFrame {
            $(
                pub $field: u32,
            )*
            $(
                $( #[$cfg] pub $cfg_field: u32, )*
            )*
        }
    };
}

critical_frame! {
    {
        mepc,
        ra,
        sp,
        gp,
        tp,
        t0, t1, t2, t3, t4, t5, t6,
        s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11,
        a0, a1, a2, a3, a4, a5, a6, a7,
        mstatus,
        mtvec,
        mcause,
        mtval,
        mie,
        mip,
        pmufunc,
    },
    #[cfg(any(esp32p4))]
    {
        mtvt,
    },
    #[cfg(any(cpu_retention_frame = "clic", esp32p4, esp32s31))]
    {
        mintthresh,
    },
    #[cfg(any(esp32p4, esp32s31))]
    {
        fpu_ft0, fpu_ft1, fpu_ft2, fpu_ft3, fpu_ft4, fpu_ft5, fpu_ft6, fpu_ft7,
        fpu_ft8, fpu_ft9, fpu_ft10, fpu_ft11,
        fpu_fs0, fpu_fs1, fpu_fs2, fpu_fs3, fpu_fs4, fpu_fs5, fpu_fs6, fpu_fs7,
        fpu_fs8, fpu_fs9, fpu_fs10, fpu_fs11,
        fpu_fa0, fpu_fa1, fpu_fa2, fpu_fa3, fpu_fa4, fpu_fa5, fpu_fa6, fpu_fa7,
        fpu_fcsr,
    }
}

/// `RV_SLEEP_CTX_FRMSZ`: the critical frame size, rounded up to 16 bytes.
pub(crate) const CRITICAL_FRAME_SIZE: usize = size_of::<CriticalSleepFrame>().next_multiple_of(16);

/// Declares the non-critical frame and the matched pair that moves it.
///
/// One list drives the struct, the save and the restore, so the three cannot drift apart.
macro_rules! non_critical_frame {
    ($($field:ident = $csr:expr),* $(,)?) => {
        /// Registers that Rust saves, because the CPU is usable while they move.
        #[repr(C)]
        pub(crate) struct NonCriticalSleepFrame {
            $($field: u32,)*
        }

        impl NonCriticalSleepFrame {
            /// Reads every CSR of the frame.
            ///
            /// # Safety
            ///
            /// The caller must own the CPU state that the frame describes.
            pub(crate) unsafe fn save(&mut self) {
                $(self.$field = unsafe { read_csr!($csr) };)*
            }

            /// Writes every CSR of the frame back.
            ///
            /// # Safety
            ///
            /// The frame must hold a state that [`Self::save`] read from this core.
            pub(crate) unsafe fn restore(&self) {
                $(unsafe { write_csr!($csr, self.$field) };)*
            }
        }
    };
}

pub(crate) use non_critical_frame;

#[cfg_attr(cpu_retention_frame = "c6_h2", path = "c6_h2.rs")]
#[cfg_attr(cpu_retention_frame = "clic", path = "clic.rs")]
#[cfg_attr(cpu_retention_frame = "p4", path = "p4.rs")]
#[cfg_attr(cpu_retention_frame = "s31", path = "s31.rs")]
pub(crate) mod chip;
