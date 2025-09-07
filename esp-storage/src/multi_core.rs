use esp_hal::{
    peripherals::LPWR,
    system::Cpu,
};

use crate::{FlashStorage, common::FlashStorageError};

/// Strategy to use on a multi core system where writing to the flash needs exclusive access from one core
#[derive(Debug)]
pub(crate) enum MultiCoreStrategy {
    /// Flash writes simply fail if the second core is active while attempting a write (default behavior)
    Error,
    /// Auto park the other core before writing. Un-park it when writing is complete
    AutoPark,
    /// Ignore that the other core is active.
    /// This is useful if the second core is known to not fetch instructions from the flash for the duration of the write.
    /// This is unsafe to use.
    Ignore,
}

impl FlashStorage {
    /// Enable auto parking of the second core before writing to flash
    pub fn multicore_auto_park(mut self) -> FlashStorage {
        self.multi_core_strategy = MultiCoreStrategy::AutoPark;
        self
    }

    /// Do not check if the second core is active before writing to flash.
    /// This is unsafe to use.
    /// Only enable this if you are sure that the second core is not fetching instructions from the flash during the write
    pub unsafe fn multicore_ignore(mut self) -> FlashStorage {
        self.multi_core_strategy = MultiCoreStrategy::Ignore;
        self
    }
}

#[inline(always)]
fn raw_core() -> usize {
    // This method must never return UNUSED_THREAD_ID_VALUE
    cfg_if::cfg_if! {
        if #[cfg(all(multi_core, riscv))] {
            riscv::register::mhartid::read()
        } else if #[cfg(all(multi_core, xtensa))] {
            (xtensa_lx::get_processor_id() & 0x2000) as usize
        } else {
            0
        }
    }
}

#[inline(always)]
fn other_core() -> Cpu {
    // This works for both RISCV and Xtensa because both
    // get_raw_core functions return zero, _or_ something
    // greater than zero; 1 in the case of RISCV and 0x2000
    // in the case of Xtensa.
    match raw_core() {
        0 => Cpu::AppCpu,
        #[cfg(all(multi_core, riscv))]
        1 => Cpu::ProCpu,
        #[cfg(all(multi_core, xtensa))]
        0x2000 => Cpu::ProCpu,
        _ => unreachable!(),
    }
}

unsafe fn park_core(core: Cpu, park: bool) {
    let c1_value = if park { 0x21 } else { 0 };
    let c0_value = if park { 0x02 } else { 0 };
    match core {
        Cpu::ProCpu => {
            LPWR::regs()
                .sw_cpu_stall()
                .modify(|_, w| unsafe { w.sw_stall_procpu_c1().bits(c1_value) });
            LPWR::regs()
                .options0()
                .modify(|_, w| unsafe { w.sw_stall_procpu_c0().bits(c0_value) });
        }
        Cpu::AppCpu => {
            LPWR::regs()
                .sw_cpu_stall()
                .modify(|_, w| unsafe { w.sw_stall_appcpu_c1().bits(c1_value) });
            LPWR::regs()
                .options0()
                .modify(|_, w| unsafe { w.sw_stall_appcpu_c0().bits(c0_value) });
        }
    }
}

fn is_cpu_running(core: Cpu) -> bool {
    match core {
        Cpu::ProCpu => {
            // ?
        }
        Cpu::AppCpu => {
            // ?
        }
    }
    true
}

impl MultiCoreStrategy {
    fn pre_write(&self) -> Result<bool, FlashStorageError> {
        match self {
            MultiCoreStrategy::Error => {
                let other_cpu = other_core();
                if is_cpu_running(other_cpu) {
                    Err(FlashStorageError::SecondCoreRunning)
                } else {
                    Ok(false)
                }
            }
            MultiCoreStrategy::AutoPark => {
                let other_cpu = other_core();

                if is_cpu_running(other_cpu) {
                    unsafe {
                        park_core(other_cpu, true);
                    }
                    Ok(true)
                } else {
                    Ok(false)
                }
            }
            MultiCoreStrategy::Ignore => Ok(false),
        }
    }

    fn post_write(&self, unpark: bool) {
        match self {
            MultiCoreStrategy::AutoPark => {
                if unpark {
                    unsafe {
                        park_core(other_core(), false);
                    }
                }
            }
            _ => {}
        }
    }

    pub(crate) fn with_checks(
        &self,
        operation: impl FnOnce() -> Result<(), FlashStorageError>,
    ) -> Result<(), FlashStorageError> {
        let res = self.pre_write()?;
        operation()?;
        self.post_write(res);
        Ok(())
    }
}
