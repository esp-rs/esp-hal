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
};

use crate::{
    cpu::Cpu,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{CPU_CTRL, LPWR, SYSTEM},
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
static mut START_CORE1_FUNCTION: Option<*mut ()> = None;
static mut APP_CORE_STACK_TOP: Option<*mut u32> = None;

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

/// Control CPU Cores
///
/// ## Examples
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use esp_hal::delay::Delay;
/// # use esp_hal::cpu_control::{CpuControl, Stack};
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
/// let _guard = cpu_control
///     .start_app_core(
///         unsafe { &mut *addr_of_mut!(APP_CORE_STACK) },
///         cpu1_fnctn
///     )?;
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
/// fn cpu1_task(
///     delay: &Delay,
///     counter: &critical_section::Mutex<RefCell<i32>>,
/// ) -> ! {
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
    _cpu_control: PeripheralRef<'d, CPU_CTRL>,
}

unsafe fn internal_park_core(core: Cpu, park: bool) {
    let c1_value = if park { 0x21 } else { 0 };
    let c0_value = if park { 0x02 } else { 0 };
    match core {
        Cpu::ProCpu => {
            LPWR::regs()
                .sw_cpu_stall()
                .modify(|_, w| w.sw_stall_procpu_c1().bits(c1_value));
            LPWR::regs()
                .options0()
                .modify(|_, w| w.sw_stall_procpu_c0().bits(c0_value));
        }
        Cpu::AppCpu => {
            LPWR::regs()
                .sw_cpu_stall()
                .modify(|_, w| w.sw_stall_appcpu_c1().bits(c1_value));
            LPWR::regs()
                .options0()
                .modify(|_, w| w.sw_stall_appcpu_c0().bits(c0_value));
        }
    }
}

impl<'d> CpuControl<'d> {
    /// Creates a new instance of `CpuControl`.
    #[instability::unstable]
    pub fn new(cpu_control: impl Peripheral<P = CPU_CTRL> + 'd) -> CpuControl<'d> {
        crate::into_ref!(cpu_control);

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
        internal_park_core(core, true);
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
        xtensa_lx::interrupt::set_mask(0);

        // reset cycle compare registers
        xtensa_lx::timer::set_ccompare0(0);
        xtensa_lx::timer::set_ccompare1(0);
        xtensa_lx::timer::set_ccompare2(0);

        extern "C" {
            static mut _init_start: u32;
        }

        unsafe {
            // move vec table
            let base = core::ptr::addr_of!(_init_start);
            core::arch::asm!("wsr.vecbase {0}", in(reg) base, options(nostack));
        }

        // switch to new stack
        xtensa_lx::set_stack_pointer(unwrap!(APP_CORE_STACK_TOP));

        // Trampoline to run from the new stack.
        // start_core1_run should _NEVER_ be inlined
        // as we rely on the function call to use
        // the new stack.
        Self::start_core1_run::<F>()
    }

    /// Run the core1 closure.
    #[inline(never)]
    unsafe fn start_core1_run<F>() -> !
    where
        F: FnOnce(),
    {
        #[allow(static_mut_refs)] // FIXME
        match START_CORE1_FUNCTION.take() {
            Some(entry) => {
                let entry = unsafe { ManuallyDrop::take(&mut *entry.cast::<ManuallyDrop<F>>()) };
                entry();
                loop {
                    unsafe { internal_park_core(Cpu::current(), true) };
                }
            }
            None => panic!("No start function set"),
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
            START_CORE1_FUNCTION = Some(entry_fn);
            APP_CORE_STACK_TOP = Some(stack.top());
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
