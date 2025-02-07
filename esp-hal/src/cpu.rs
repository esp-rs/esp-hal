//! CPU related functionality

/// Available CPU cores
///
/// The actual number of available cores depends on the target.
#[derive(Debug, Copy, Clone, PartialEq, Eq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C)]
pub enum Cpu {
    /// The first core
    ProCpu = 0,
    /// The second core
    #[cfg(multi_core)]
    AppCpu = 1,
}

impl Cpu {
    /// The number of available cores.
    pub const COUNT: usize = 1 + cfg!(multi_core) as usize;

    /// Returns the core the application is currently executing on
    #[inline(always)]
    pub fn current() -> Self {
        // This works for both RISCV and Xtensa because both
        // get_raw_core functions return zero, _or_ something
        // greater than zero; 1 in the case of RISCV and 0x2000
        // in the case of Xtensa.
        match raw_core() {
            0 => Cpu::ProCpu,
            #[cfg(all(multi_core, riscv))]
            1 => Cpu::AppCpu,
            #[cfg(all(multi_core, xtensa))]
            0x2000 => Cpu::AppCpu,
            _ => unreachable!(),
        }
    }

    /// Returns an iterator over the "other" cores.
    #[inline(always)]
    pub(crate) fn other() -> impl Iterator<Item = Self> {
        cfg_if::cfg_if! {
            if #[cfg(multi_core)] {
                match Self::current() {
                    Cpu::ProCpu => [Cpu::AppCpu].into_iter(),
                    Cpu::AppCpu => [Cpu::ProCpu].into_iter(),
                }
            } else {
                [].into_iter()
            }
        }
    }

    /// Returns an iterator over all cores.
    #[inline(always)]
    pub(crate) fn all() -> impl Iterator<Item = Self> {
        cfg_if::cfg_if! {
            if #[cfg(multi_core)] {
                [Cpu::ProCpu, Cpu::AppCpu].into_iter()
            } else {
                [Cpu::ProCpu].into_iter()
            }
        }
    }
}

/// Returns the raw value of the mhartid register.
///
/// Safety: This method should never return UNUSED_THREAD_ID_VALUE
#[cfg(riscv)]
#[inline(always)]
pub(crate) fn raw_core() -> usize {
    #[cfg(multi_core)]
    {
        riscv::register::mhartid::read()
    }

    #[cfg(not(multi_core))]
    0
}

/// Returns the result of reading the PRID register logically ANDed with 0x2000,
/// the 13th bit in the register. Espressif Xtensa chips use this bit to
/// determine the core id.
///
/// Returns either 0 or 0x2000
///
/// Safety: This method should never return UNUSED_THREAD_ID_VALUE
#[cfg(xtensa)]
#[inline(always)]
pub(crate) fn raw_core() -> usize {
    (xtensa_lx::get_processor_id() & 0x2000) as usize
}
