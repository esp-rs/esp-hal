//! # System Control

use esp_sync::NonReentrantMutex;

// Implements the Peripheral enum based on esp-metadata/device.soc/peripheral_clocks
implement_peripheral_clocks!();

impl Peripheral {
    pub const fn try_from(value: u8) -> Option<Peripheral> {
        if value >= Peripheral::COUNT as u8 {
            return None;
        }

        Some(unsafe { core::mem::transmute::<u8, Peripheral>(value) })
    }
}

struct RefCounts {
    counts: [usize; Peripheral::COUNT],
}

impl RefCounts {
    pub const fn new() -> Self {
        Self {
            counts: [0; Peripheral::COUNT],
        }
    }
}

static PERIPHERAL_REF_COUNT: NonReentrantMutex<RefCounts> =
    NonReentrantMutex::new(RefCounts::new());

/// Disable all peripherals.
///
/// Peripherals listed in [KEEP_ENABLED] are NOT disabled.
#[cfg_attr(not(feature = "rt"), expect(dead_code))]
pub(crate) fn disable_peripherals() {
    // Take the critical section up front to avoid taking it multiple times.
    PERIPHERAL_REF_COUNT.with(|refcounts| {
        for p in Peripheral::KEEP_ENABLED {
            refcounts.counts[*p as usize] += 1;
        }
        for p in Peripheral::ALL {
            let ref_count = refcounts.counts[*p as usize];
            if ref_count == 0 {
                PeripheralClockControl::enable_forced_with_counts(*p, false, true, refcounts);
            }
        }
    })
}

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct PeripheralGuard {
    peripheral: Peripheral,
}

impl PeripheralGuard {
    pub(crate) fn new_with(p: Peripheral, init: fn()) -> Self {
        if PeripheralClockControl::enable(p) {
            PeripheralClockControl::reset(p);
            init();
        }

        Self { peripheral: p }
    }

    pub(crate) fn new(p: Peripheral) -> Self {
        Self::new_with(p, || {})
    }
}

impl Clone for PeripheralGuard {
    fn clone(&self) -> Self {
        Self::new(self.peripheral)
    }

    fn clone_from(&mut self, _source: &Self) {
        // This is a no-op since the ref count for P remains the same.
    }
}

impl Drop for PeripheralGuard {
    fn drop(&mut self) {
        PeripheralClockControl::disable(self.peripheral);
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct GenericPeripheralGuard<const P: u8> {}

impl<const P: u8> GenericPeripheralGuard<P> {
    pub(crate) fn new_with(init: fn()) -> Self {
        let peripheral = const { Peripheral::try_from(P).unwrap() };

        PERIPHERAL_REF_COUNT.with(|ref_counts| {
            if PeripheralClockControl::enable_with_counts(peripheral, ref_counts) {
                unsafe { PeripheralClockControl::reset_racey(peripheral) };
                init();
            }
        });

        Self {}
    }

    #[cfg_attr(not(feature = "unstable"), allow(unused))]
    pub(crate) fn new() -> Self {
        Self::new_with(|| {})
    }
}

impl<const P: u8> Clone for GenericPeripheralGuard<P> {
    fn clone(&self) -> Self {
        Self::new()
    }

    fn clone_from(&mut self, _source: &Self) {
        // This is a no-op since the ref count for P remains the same.
    }
}

impl<const P: u8> Drop for GenericPeripheralGuard<P> {
    fn drop(&mut self) {
        let peripheral = const { Peripheral::try_from(P).unwrap() };
        PeripheralClockControl::disable(peripheral);
    }
}

/// Controls the enablement of peripheral clocks.
pub(crate) struct PeripheralClockControl;

impl PeripheralClockControl {
    /// Enables the given peripheral.
    ///
    /// This keeps track of enabling a peripheral - i.e. a peripheral
    /// is only enabled with the first call attempt to enable it.
    ///
    /// Returns `true` if it actually enabled the peripheral.
    pub(crate) fn enable(peripheral: Peripheral) -> bool {
        PERIPHERAL_REF_COUNT.with(|ref_counts| Self::enable_with_counts(peripheral, ref_counts))
    }

    /// Enables the given peripheral.
    ///
    /// This keeps track of enabling a peripheral - i.e. a peripheral
    /// is only enabled with the first call attempt to enable it.
    ///
    /// Returns `true` if it actually enabled the peripheral.
    fn enable_with_counts(peripheral: Peripheral, ref_counts: &mut RefCounts) -> bool {
        Self::enable_forced_with_counts(peripheral, true, false, ref_counts)
    }

    /// Disables the given peripheral.
    ///
    /// This keeps track of disabling a peripheral - i.e. it only
    /// gets disabled when the number of enable/disable attempts is balanced.
    ///
    /// Returns `true` if it actually disabled the peripheral.
    ///
    /// Before disabling a peripheral it will also get reset
    pub(crate) fn disable(peripheral: Peripheral) -> bool {
        PERIPHERAL_REF_COUNT.with(|ref_counts| {
            Self::enable_forced_with_counts(peripheral, false, false, ref_counts)
        })
    }

    fn enable_forced_with_counts(
        peripheral: Peripheral,
        enable: bool,
        force: bool,
        ref_counts: &mut RefCounts,
    ) -> bool {
        let ref_count = &mut ref_counts.counts[peripheral as usize];
        if !force {
            let prev = *ref_count;
            if enable {
                *ref_count += 1;
                trace!("Enable {:?} {} -> {}", peripheral, prev, *ref_count);
                if prev > 0 {
                    return false;
                }
            } else {
                assert!(prev != 0);
                *ref_count -= 1;
                trace!("Disable {:?} {} -> {}", peripheral, prev, *ref_count);
                if prev > 1 {
                    return false;
                }
            };
        } else if !enable {
            assert!(*ref_count == 0);
        }

        if !enable {
            unsafe { Self::reset_racey(peripheral) };
        }

        debug!("Enable {:?} {}", peripheral, enable);
        unsafe { enable_internal_racey(peripheral, enable) };

        true
    }

    /// Resets the given peripheral
    pub(crate) unsafe fn reset_racey(peripheral: Peripheral) {
        debug!("Reset {:?}", peripheral);

        unsafe {
            assert_peri_reset_racey(peripheral, true);
            assert_peri_reset_racey(peripheral, false);
        }
    }

    /// Resets the given peripheral
    pub(crate) fn reset(peripheral: Peripheral) {
        PERIPHERAL_REF_COUNT.with(|_| unsafe { Self::reset_racey(peripheral) })
    }
}

#[cfg(any(esp32, esp32s3))]
#[allow(unused_imports)]
pub use crate::soc::cpu_control::*;

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

    #[procmacros::doc_replace]
    /// Returns the core the application is currently executing on
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// #
    /// use esp_hal::system::Cpu;
    /// let current_cpu = Cpu::current();
    /// #
    /// # {after_snippet}
    /// ```
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
    #[instability::unstable]
    pub fn other() -> impl Iterator<Item = Self> {
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
/// On RISC-V, this is the hardware thread ID.
///
/// On Xtensa, this returns the result of reading the PRID register logically
/// ANDed with 0x2000, the 13th bit in the register. Espressif Xtensa chips use
/// this bit to determine the core id.
#[inline(always)]
pub(crate) fn raw_core() -> usize {
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

use crate::rtc_cntl::SocResetReason;

/// Source of the wakeup event
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum SleepSource {
    /// In case of deep sleep, reset was not caused by exit from deep sleep
    Undefined = 0,
    /// Not a wakeup cause, used to disable all wakeup sources with
    /// esp_sleep_disable_wakeup_source
    All,
    /// Wakeup caused by external signal using RTC_IO
    Ext0,
    /// Wakeup caused by external signal using RTC_CNTL
    Ext1,
    /// Wakeup caused by timer
    Timer,
    /// Wakeup caused by touchpad
    TouchPad,
    /// Wakeup caused by ULP program
    Ulp,
    /// Wakeup caused by GPIO (light sleep only on ESP32, S2 and S3)
    Gpio,
    /// Wakeup caused by UART (light sleep only)
    Uart,
    /// Wakeup caused by WIFI (light sleep only)
    Wifi,
    /// Wakeup caused by COCPU int
    Cocpu,
    /// Wakeup caused by COCPU crash
    CocpuTrapTrig,
    /// Wakeup caused by BT (light sleep only)
    BT,
}

#[procmacros::doc_replace]
/// Performs a software reset on the chip.
///
/// # Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::system::software_reset;
/// software_reset();
/// # {after_snippet}
/// ```
#[inline]
pub fn software_reset() -> ! {
    crate::rom::software_reset()
}

/// Resets the given CPU, leaving peripherals unchanged.
#[instability::unstable]
#[inline]
pub fn software_reset_cpu(cpu: Cpu) {
    crate::rom::software_reset_cpu(cpu as u32)
}

/// Retrieves the reason for the last reset as a SocResetReason enum value.
/// Returns `None` if the reset reason cannot be determined.
#[instability::unstable]
#[inline]
pub fn reset_reason() -> Option<SocResetReason> {
    crate::rtc_cntl::reset_reason(Cpu::current())
}

/// Retrieves the cause of the last wakeup event as a SleepSource enum value.
#[instability::unstable]
#[inline]
pub fn wakeup_cause() -> SleepSource {
    crate::rtc_cntl::wakeup_cause()
}
