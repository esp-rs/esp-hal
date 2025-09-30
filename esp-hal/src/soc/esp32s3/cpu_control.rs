//! # Control CPU Cores (ESP32-S3)
//!
//! ## Overview
//!
//! This module provides essential functionality for controlling
//! and managing the APP (second) CPU core on the `ESP32-S3` chip. It is used to
//! start and stop program execution on the APP core.

use core::{
    marker::PhantomData,
    mem::{ManuallyDrop, MaybeUninit},
    sync::atomic::{AtomicPtr, Ordering},
};

use crate::{
    peripherals::{CPU_CTRL, LPWR, SYSTEM},
    system::Cpu,
};

/// Data type for a properly aligned stack of N bytes
// Xtensa ISA 10.5: [B]y default, the
// stack frame is 16-byte aligned. However, the maximal alignment allowed for a
// TIE ctype is 64-bytes. If a function has any wide-aligned (>16-byte aligned)
// data type for their arguments or the return values, the caller has to ensure
// that the SP is aligned to the largest alignment right before the call.
//
// ^ this means that we should be able to get away with 16 bytes of alignment
// because our root stack frame has no arguments and no return values.
//
// This alignment also doesn't align the stack frames, only the end of stack.
// Stack frame alignment depends on the SIZE as well as the placement of the
// array.
#[repr(C, align(16))]
#[instability::unstable]
pub struct Stack<const SIZE: usize> {
    /// Memory to be used for the stack
    pub mem: MaybeUninit<[u8; SIZE]>,
}

impl<const SIZE: usize> Default for Stack<SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

#[allow(clippy::len_without_is_empty)]
impl<const SIZE: usize> Stack<SIZE> {
    /// Construct a stack of length SIZE, uninitialized
    #[instability::unstable]
    pub const fn new() -> Stack<SIZE> {
        ::core::assert!(SIZE % 16 == 0); // Make sure stack top is aligned, too.

        Stack {
            mem: MaybeUninit::uninit(),
        }
    }

    /// Returns the length of the stack in bytes.
    #[instability::unstable]
    pub const fn len(&self) -> usize {
        SIZE
    }

    /// Returns a mutable pointer to the bottom of the stack.
    #[instability::unstable]
    pub fn bottom(&mut self) -> *mut u32 {
        self.mem.as_mut_ptr() as *mut u32
    }

    /// Returns a mutable pointer to the top of the stack.
    #[instability::unstable]
    pub fn top(&mut self) -> *mut u32 {
        unsafe { self.bottom().add(SIZE / 4) }
    }
}

// Pointer to the closure that will be executed on the second core. The closure
// is copied to the core's stack.
static START_CORE1_FUNCTION: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());
static APP_CORE_STACK_TOP: AtomicPtr<u32> = AtomicPtr::new(core::ptr::null_mut());
static APP_CORE_STACK_GUARD: AtomicPtr<u32> = AtomicPtr::new(core::ptr::null_mut());

/// Will park the APP (second) core when dropped
#[must_use = "Dropping this guard will park the APP core"]
#[instability::unstable]
pub struct AppCoreGuard<'a> {
    phantom: PhantomData<&'a ()>,
}

impl Drop for AppCoreGuard<'_> {
    fn drop(&mut self) {
        unsafe { internal_park_core(Cpu::AppCpu, true) };
    }
}

/// Represents errors that can occur while working with the core.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum Error {
    /// The core is already running.
    CoreAlreadyRunning,
}

#[procmacros::doc_replace]
/// Control CPU Cores
///
/// ## Examples
/// ```rust, no_run
/// # {before_snippet}
/// # use esp_hal::delay::Delay;
/// # use esp_hal::system::{CpuControl, Stack};
/// # use core::{cell::RefCell, ptr::addr_of_mut};
/// # use critical_section::Mutex;
/// # let delay = Delay::new();
/// static mut APP_CORE_STACK: Stack<8192> = Stack::new();
///
/// let counter = Mutex::new(RefCell::new(0));
///
/// let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
/// let cpu1_fnctn = || {
///     cpu1_task(&delay, &counter);
/// };
/// let _guard =
///     cpu_control.start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, cpu1_fnctn)?;
///
/// loop {
///     delay.delay(Duration::from_secs(1));
///     let count = critical_section::with(|cs| *counter.borrow_ref(cs));
/// }
/// # }
///
/// // Where `cpu1_task()` may be defined as:
/// # use esp_hal::delay::Delay;
/// # use core::cell::RefCell;
///
/// fn cpu1_task(delay: &Delay, counter: &critical_section::Mutex<RefCell<i32>>) -> ! {
///     loop {
///         delay.delay(Duration::from_millis(500));
///
///         critical_section::with(|cs| {
///             let mut val = counter.borrow_ref_mut(cs);
///             *val = val.wrapping_add(1);
///         });
///     }
/// }
/// ```
#[instability::unstable]
pub struct CpuControl<'d> {
    _cpu_control: CPU_CTRL<'d>,
}

unsafe fn internal_park_core(core: Cpu, park: bool) {
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

/// Returns `true` if the specified core is currently running (not stalled).
#[instability::unstable]
pub fn is_running(core: Cpu) -> bool {
    if core == Cpu::AppCpu {
        // CORE_1_RUNSTALL in bit 0 -> needs to be 0 to not stall
        // CORE_1_CLKGATE_EN in bit 1 -> needs to be 1 to even be enabled
        let system = SYSTEM::regs();
        let r = system.core_1_control_0().read();
        if r.control_core_1_clkgate_en().bit_is_clear() || r.control_core_1_runstall().bit_is_set()
        {
            // If the core is not enabled we can take this shortcut
            return false;
        }
    }

    // sw_stall_appcpu_c1[5:0],  sw_stall_appcpu_c0[1:0]} == 0x86 will stall APP CPU
    // sw_stall_procpu_c1[5:0],  reg_sw_stall_procpu_c0[1:0]} == 0x86 will stall PRO CPU
    let is_stalled = match core {
        Cpu::ProCpu => {
            let c1 = LPWR::regs()
                .sw_cpu_stall()
                .read()
                .sw_stall_procpu_c1()
                .bits();
            let c0 = LPWR::regs().options0().read().sw_stall_procpu_c0().bits();
            (c1 << 2) | c0
        }
        Cpu::AppCpu => {
            let c1 = LPWR::regs()
                .sw_cpu_stall()
                .read()
                .sw_stall_appcpu_c1()
                .bits();
            let c0 = LPWR::regs().options0().read().sw_stall_appcpu_c0().bits();
            (c1 << 2) | c0
        }
    };

    is_stalled != 0x86
}

impl<'d> CpuControl<'d> {
    /// Creates a new instance of `CpuControl`.
    #[instability::unstable]
    pub fn new(cpu_control: CPU_CTRL<'d>) -> CpuControl<'d> {
        CpuControl {
            _cpu_control: cpu_control,
        }
    }

    /// Park the given core
    ///
    /// # Safety
    ///
    /// The user must ensure that the core being parked is not the core which is
    /// currently executing their code.
    #[instability::unstable]
    pub unsafe fn park_core(&mut self, core: Cpu) {
        unsafe {
            internal_park_core(core, true);
        }
    }

    /// Unpark the given core
    #[instability::unstable]
    pub fn unpark_core(&mut self, core: Cpu) {
        unsafe { internal_park_core(core, false) };
    }

    /// When we get here, the core is out of reset, with a stack setup by ROM
    /// code
    ///
    /// We need to initialize the CPU fully, and setup the new stack to use the
    /// stack provided by the user.
    unsafe fn start_core1_init<F>() -> !
    where
        F: FnOnce(),
    {
        // disables interrupts
        unsafe {
            xtensa_lx::interrupt::set_mask(0);
        }

        // reset cycle compare registers
        xtensa_lx::timer::set_ccompare0(0);
        xtensa_lx::timer::set_ccompare1(0);
        xtensa_lx::timer::set_ccompare2(0);

        unsafe extern "C" {
            static mut _init_start: u32;
        }

        // set vector table and stack pointer
        unsafe {
            xtensa_lx::set_vecbase(&raw const _init_start);
            xtensa_lx::set_stack_pointer(APP_CORE_STACK_TOP.load(Ordering::Acquire));

            #[cfg(all(feature = "rt", stack_guard_monitoring))]
            {
                let stack_guard = APP_CORE_STACK_GUARD.load(Ordering::Acquire);
                stack_guard.write_volatile(esp_config::esp_config_int!(
                    u32,
                    "ESP_HAL_CONFIG_STACK_GUARD_VALUE"
                ));
                // setting 0 effectively disables the functionality
                crate::debugger::set_stack_watchpoint(stack_guard as usize);
            }
        }

        // Trampoline to run from the new stack.
        // start_core1_run should _NEVER_ be inlined
        // as we rely on the function call to use
        // the new stack.
        unsafe { Self::start_core1_run::<F>() }
    }

    /// Run the core1 closure.
    #[inline(never)]
    unsafe fn start_core1_run<F>() -> !
    where
        F: FnOnce(),
    {
        let entry = START_CORE1_FUNCTION.load(Ordering::Acquire);
        debug_assert!(!entry.is_null());

        unsafe {
            let entry = ManuallyDrop::take(&mut *entry.cast::<ManuallyDrop<F>>());
            entry();
            loop {
                internal_park_core(Cpu::current(), true);
            }
        }
    }

    /// Start the APP (second) core.
    ///
    /// The second core will start running the closure `entry`. Note that if the
    /// closure exits, the core will be parked.
    ///
    /// Dropping the returned guard will park the core.
    #[instability::unstable]
    pub fn start_app_core<'a, const SIZE: usize, F>(
        &mut self,
        stack: &'static mut Stack<SIZE>,
        entry: F,
    ) -> Result<AppCoreGuard<'a>, Error>
    where
        F: FnOnce(),
        F: Send + 'a,
    {
        cfg_if::cfg_if! {
            if #[cfg(all(stack_guard_monitoring))] {
                let stack_guard_offset = Some(esp_config::esp_config_int!(
                    usize,
                    "ESP_HAL_CONFIG_STACK_GUARD_OFFSET"
                ));
            } else {
                let stack_guard_offset = None;
            }
        };

        self.start_app_core_with_stack_guard_offset(stack, stack_guard_offset, entry)
    }

    /// Start the APP (second) core.
    ///
    /// The second core will start running the closure `entry`. Note that if the
    /// closure exits, the core will be parked.
    ///
    /// Dropping the returned guard will park the core.
    #[instability::unstable]
    pub fn start_app_core_with_stack_guard_offset<'a, const SIZE: usize, F>(
        &mut self,
        stack: &'static mut Stack<SIZE>,
        stack_guard_offset: Option<usize>,
        entry: F,
    ) -> Result<AppCoreGuard<'a>, Error>
    where
        F: FnOnce(),
        F: Send + 'a,
    {
        let system_control = SYSTEM::regs();

        if !xtensa_lx::is_debugger_attached()
            && system_control
                .core_1_control_0()
                .read()
                .control_core_1_clkgate_en()
                .bit_is_set()
        {
            return Err(Error::CoreAlreadyRunning);
        }

        // We don't want to drop this, since it's getting moved to the other core.
        let entry = ManuallyDrop::new(entry);

        unsafe {
            let stack_bottom = stack.bottom().cast::<u8>();

            // Push `entry` to an aligned address at the (physical) bottom of the stack.
            // The second core will copy it into its proper place, then calls it.
            let align_offset = stack_bottom.align_offset(core::mem::align_of::<F>());
            let entry_dst = stack_bottom.add(align_offset).cast::<ManuallyDrop<F>>();

            entry_dst.write(entry);

            let entry_fn = entry_dst.cast::<()>();
            START_CORE1_FUNCTION.store(entry_fn, Ordering::Release);
            APP_CORE_STACK_TOP.store(stack.top(), Ordering::Release);
            let stack_guard = if let Some(stack_guard_offset) = stack_guard_offset {
                assert!(stack_guard_offset.is_multiple_of(4));
                assert!(stack_guard_offset <= stack.len() - 4);
                stack_bottom.byte_add(stack_guard_offset)
            } else {
                core::ptr::null_mut()
            };
            APP_CORE_STACK_GUARD.store(stack_guard.cast(), Ordering::Release);
        }

        crate::rom::ets_set_appcpu_boot_addr(Self::start_core1_init::<F> as *const u32 as u32);

        system_control
            .core_1_control_0()
            .modify(|_, w| w.control_core_1_clkgate_en().set_bit());
        system_control
            .core_1_control_0()
            .modify(|_, w| w.control_core_1_runstall().clear_bit());
        system_control
            .core_1_control_0()
            .modify(|_, w| w.control_core_1_reseting().set_bit());
        system_control
            .core_1_control_0()
            .modify(|_, w| w.control_core_1_reseting().clear_bit());

        self.unpark_core(Cpu::AppCpu);

        Ok(AppCoreGuard {
            phantom: PhantomData,
        })
    }
}
