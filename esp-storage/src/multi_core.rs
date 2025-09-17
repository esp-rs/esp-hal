//! Only available/needed on multi-core systems like the ESP32-S3 and ESP32

use crate::{FlashStorage, common::FlashStorageError};

#[cfg(esp32)]
mod registers {
    pub(crate) const OPTIONS0: u32 = 0x3ff4_8000;
    pub(crate) const SW_CPU_STALL: u32 = 0x3ff4_80ac;
    pub(crate) const APPCPU_CTRL_B: u32 = 0x3FF0_0030;
    pub(crate) const APPCPU_CTRL_C: u32 = 0x3FF0_0034;
}

#[cfg(esp32s3)]
mod registers {
    pub(crate) const OPTIONS0: u32 = 0x6000_8000;
    pub(crate) const SW_CPU_STALL: u32 = 0x6000_80bc;
    pub(crate) const CORE_1_CONTROL_0: u32 = 0x600c_0000;
}

const C0_VALUE_PRO: u32 = 0x02 << 2;
const C1_VALUE_PRO: u32 = 0x21 << 26;
const C0_VALUE_APP: u32 = 0x02;
const C1_VALUE_APP: u32 = 0x21 << 20;

/// Strategy to use on a multi core system where writing to the flash needs exclusive access from
/// one core
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum MultiCoreStrategy {
    /// Flash writes simply fail if the second core is active while attempting a write (default
    /// behavior)
    Error,
    /// Auto park the other core before writing. Un-park it when writing is complete
    AutoPark,
    /// Ignore that the other core is active.
    /// This is useful if the second core is known to not fetch instructions from the flash for the
    /// duration of the write. This is unsafe to use.
    Ignore,
}

impl FlashStorage {
    /// Enable auto parking of the second core before writing to flash
    /// The other core will be automatically un-parked when the write is complete
    pub fn multicore_auto_park(mut self) -> FlashStorage {
        self.multi_core_strategy = MultiCoreStrategy::AutoPark;
        self
    }

    /// Do not check if the second core is active before writing to flash.
    ///
    /// # Safety
    /// Only enable this if you are sure that the second core is not fetching instructions from the
    /// flash during the write
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

#[derive(Clone, Copy, Debug)]
pub enum Cpu {
    /// The first core
    ProCpu = 0,
    /// The second core
    AppCpu = 1,
}

impl Cpu {
    /// Returns the core other than the one which this function is called on
    #[inline(always)]
    fn other() -> Cpu {
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

    #[inline(always)]
    fn get_c0_c1_bits(self) -> (u32, u32) {
        match self {
            Cpu::ProCpu => (C0_VALUE_PRO, C1_VALUE_PRO),
            Cpu::AppCpu => (C0_VALUE_APP, C1_VALUE_APP),
        }
    }

    /// Park or un-park the core
    #[inline(always)]
    fn park_core(self, park: bool) {
        let sw_cpu_stall = registers::SW_CPU_STALL as *mut u32;
        let options0 = registers::OPTIONS0 as *mut u32;

        // Write 0x2 to options0 to stall a particular core
        // offset for app cpu: 0
        // offset for pro cpu: 2

        // Write 0x21 to sw_cpu_stall to allow stalling of a particular core
        // offset for app cpu: 20
        // offset for pro cpu: 26

        let (c0_bits, c1_bits) = self.get_c0_c1_bits();
        unsafe {
            let mut c0 = options0.read_volatile() & !(c0_bits);
            let mut c1 = sw_cpu_stall.read_volatile() & !(c1_bits);
            if park {
                c0 |= c0_bits;
                c1 |= c1_bits;
            }
            sw_cpu_stall.write_volatile(c1);
            options0.write_volatile(c0);
        }
    }

    /// Returns true if the core is running
    #[inline(always)]
    fn is_running(&self) -> bool {
        // If the core is the app cpu we need to check first if it was even enabled
        if let Cpu::AppCpu = *self {
            cfg_if::cfg_if! {
                if #[cfg(esp32s3)] {
                    // CORE_1_RUNSTALL in bit 0 -> needs to be 0 to not stall
                    // CORE_1_CLKGATE_EN in bit 1 -> needs to be 1 to even be enabled
                    let core_1_control_0 = registers::CORE_1_CONTROL_0 as *mut u32;
                    if unsafe { core_1_control_0.read_volatile() } & 0x03 != 0x02 {
                        // If the core is not enabled we can take this shortcut
                        return false;
                    }
                } else if #[cfg(esp32)] {
                    // DPORT_APPCPU_CLKGATE_EN in APPCPU_CTRL_B bit 0 -> needs to be 1 to even be enabled
                    // DPORT_APPCPU_RUNSTALL in APPCPU_CTRL_C bit 0 -> needs to be 0 to not stall
                    let appcpu_ctrl_b = registers::APPCPU_CTRL_B as *mut u32;
                    if unsafe { appcpu_ctrl_b.read_volatile() } & 0x01 != 0x01 {
                        // If the core is not enabled we can take this shortcut
                        return false;
                    }
                    let appcpu_ctrl_c = registers::APPCPU_CTRL_C as *mut u32;
                    if unsafe { appcpu_ctrl_c.read_volatile() } & 0x01 == 0x01 {
                        // If the core is stalled we can take this shortcut
                        return false;
                    }
                }
            }
        }

        let sw_cpu_stall = registers::SW_CPU_STALL as *mut u32;
        let options0 = registers::OPTIONS0 as *mut u32;

        let (c0_bits, c1_bits) = self.get_c0_c1_bits();
        let (c0, c1) = unsafe {
            (
                options0.read_volatile() & c0_bits,
                sw_cpu_stall.read_volatile() & c1_bits,
            )
        };
        !(c0 == c0_bits && c1 == c1_bits)
    }
}

impl MultiCoreStrategy {
    /// Perform checks/Prepare for a flash write according to the current strategy
    ///
    /// # Returns
    /// * `True` if the other core needs to be un-parked by post_write
    /// * `False` otherwise
    pub(crate) fn pre_write(&self) -> Result<bool, FlashStorageError> {
        match self {
            MultiCoreStrategy::Error => {
                if Cpu::other().is_running() {
                    Err(FlashStorageError::OtherCoreRunning)
                } else {
                    Ok(false)
                }
            }
            MultiCoreStrategy::AutoPark => {
                let other_cpu = Cpu::other();
                if other_cpu.is_running() {
                    other_cpu.park_core(true);
                    Ok(true)
                } else {
                    Ok(false)
                }
            }
            MultiCoreStrategy::Ignore => Ok(false),
        }
    }

    /// Perform post-write actions
    ///
    /// # Returns
    /// * `True` if the other core needs to be un-parked by post_write
    /// * `False` otherwise
    pub(crate) fn post_write(&self, unpark: bool) {
        if let MultiCoreStrategy::AutoPark = self {
            if unpark {
                Cpu::other().park_core(false);
            }
        }
    }
}
