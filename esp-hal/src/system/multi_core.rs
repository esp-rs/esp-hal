//! Multi-core support

use core::{
    marker::PhantomData,
    mem::{ManuallyDrop, MaybeUninit},
    sync::atomic::{AtomicPtr, Ordering},
};

#[instability::unstable]
pub use crate::soc::cpu_control::is_running;
use crate::{peripherals::CPU_CTRL, soc::cpu_control::internal_park_core, system::Cpu};

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
    /// Creates a new stack of length SIZE, uninitialized.
    #[instability::unstable]
    pub const fn new() -> Stack<SIZE> {
        const {
            // Make sure stack top is aligned, too.
            ::core::assert!(SIZE.is_multiple_of(16));
        }

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
pub(crate) static START_CORE1_FUNCTION: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());
pub(crate) static APP_CORE_STACK_TOP: AtomicPtr<u32> = AtomicPtr::new(core::ptr::null_mut());
pub(crate) static APP_CORE_STACK_GUARD: AtomicPtr<u32> = AtomicPtr::new(core::ptr::null_mut());

/// Will park the APP (second) core when dropped.
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
/// # Examples
///
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

impl<'d> CpuControl<'d> {
    /// Creates a new instance of `CpuControl`.
    #[instability::unstable]
    pub fn new(cpu_control: CPU_CTRL<'d>) -> CpuControl<'d> {
        CpuControl {
            _cpu_control: cpu_control,
        }
    }

    /// Parks the given core.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the core being parked is not the core which is
    /// currently executing this code.
    #[instability::unstable]
    pub unsafe fn park_core(&mut self, core: Cpu) {
        unsafe { internal_park_core(core, true) };
    }

    /// Unparks the given core.
    #[instability::unstable]
    pub fn unpark_core(&mut self, core: Cpu) {
        unsafe { internal_park_core(core, false) };
    }

    /// Runs the core1 closure.
    #[inline(never)]
    pub(crate) unsafe fn start_core1_run<F>() -> !
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

    /// Starts the APP (second) core.
    ///
    /// The second core starts running the closure `entry`. If the closure exits, the core is
    /// parked.
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
        cfg_select! {
            all(stack_guard_monitoring) => {
                let stack_guard_offset = Some(esp_config::esp_config_int!(
                    usize,
                    "ESP_HAL_CONFIG_STACK_GUARD_OFFSET"
                ));
            }
            _ => {
                let stack_guard_offset = None;
            }
        };

        self.start_app_core_with_stack_guard_offset(stack, stack_guard_offset, entry)
    }

    /// Starts the APP (second) core.
    ///
    /// The second core starts running the closure `entry`. If the closure exits, the core is
    /// parked.
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
        if !crate::debugger::debugger_connected() && is_running(Cpu::AppCpu) {
            return Err(Error::CoreAlreadyRunning);
        }

        setup_second_core_stack(stack, stack_guard_offset, entry);

        crate::soc::cpu_control::start_core1(start_core1_init::<F> as *const u32);

        self.unpark_core(Cpu::AppCpu);

        Ok(AppCoreGuard {
            phantom: PhantomData,
        })
    }
}

fn setup_second_core_stack<'a, F, const SIZE: usize>(
    stack: &'static mut Stack<SIZE>,
    stack_guard_offset: Option<usize>,
    entry: F,
) where
    F: FnOnce(),
    F: Send + 'a,
{
    // We don't want to drop this, since it's getting moved to the other core.
    let entry = ManuallyDrop::new(entry);

    unsafe {
        let stack_bottom = stack.bottom().cast::<u8>();
        let (stack_guard, stack_bottom_above_guard) =
            if let Some(stack_guard_offset) = stack_guard_offset {
                assert!(stack_guard_offset.is_multiple_of(4));
                assert!(stack_guard_offset <= stack.len() - 4);
                (
                    stack_bottom.byte_add(stack_guard_offset),
                    stack_bottom.byte_add(stack_guard_offset).byte_add(4),
                )
            } else {
                (core::ptr::null_mut(), stack_bottom)
            };

        // Push `entry` to an aligned address at the (physical) bottom of the stack, but above
        // the stack guard. The second core will copy it into its proper place, then
        // calls it.
        let align_offset = stack_bottom_above_guard.align_offset(core::mem::align_of::<F>());
        let entry_dst = stack_bottom_above_guard
            .add(align_offset)
            .cast::<ManuallyDrop<F>>();

        entry_dst.write(entry);

        let entry_fn = entry_dst.cast::<()>();
        START_CORE1_FUNCTION.store(entry_fn, Ordering::Release);
        APP_CORE_STACK_TOP.store(stack.top(), Ordering::Release);
        APP_CORE_STACK_GUARD.store(stack_guard.cast(), Ordering::Release);
    }
}

/// Core 1 entry point, set as the boot address.
#[cfg(xtensa)]
pub(crate) fn start_core1_init<F>() -> !
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

    // The ROM has already handed off to us; clear the AppCpu boot address so a
    // subsequent reset doesn't see a stale entry point. Matches IDF's
    // `call_start_cpu1`.
    cfg_select! {
        esp32 => {
            crate::peripherals::DPORT::regs()
                .appcpu_ctrl_d()
                .write(|w| unsafe { w.appcpu_boot_addr().bits(0) });
        }
        _ => crate::rom::ets_set_appcpu_boot_addr(0),
    }

    // Do not call setup_interrupts as that would disable peripheral interrupts, too.
    unsafe { crate::interrupt::init_vectoring() };

    // Trampoline to run from the new stack.
    // start_core1_run should _NEVER_ be inlined
    // as we rely on the function call to use
    // the new stack.
    unsafe { CpuControl::start_core1_run::<F>() }
}

/// Core 1 entry point, set as the boot address.
///
/// ROM jumps here directly, bypassing `_start`, so `gp`, the FPU and the stack pointer need to be
/// set up before entering regular Rust.
#[cfg(riscv)]
#[unsafe(naked)]
pub(crate) extern "C" fn start_core1_init<F>() -> !
where
    F: FnOnce(),
{
    core::arch::naked_asm!(
        ".option push",
        ".option norelax",
        "la gp, __global_pointer$",
        "li ra, 0", // ensure probe-rs stops unwinding
        ".option pop",
        // Follow IDF's FPU initialization (rv_utils_enable_fpu, rv_utils_clear_fpu): enable the
        // FPU in Initial state, touch fcsr (which makes the state Dirty), then clear the dirty
        // bit to leave mstatus.FS in Clean state (0b10), which is what the interrupt handler
        // expects.
        "li t0, 0x2000",
        "csrs mstatus, t0",
        "li t0, 1",
        "csrw fcsr, t0",
        "li t0, 0x2000",
        "csrc mstatus, t0",
        // Switch to Core 1's stack (stored by Core 0 before releasing this core).
        "la t0, {stack_top}",
        "lw sp, 0(t0)",
        "j {init}",
        stack_top = sym APP_CORE_STACK_TOP,
        init = sym start_core1_init_impl::<F>,
    )
}

#[cfg(riscv)]
fn start_core1_init_impl<F>() -> !
where
    F: FnOnce(),
{
    crate::soc::enable_branch_predictor();

    // The ROM has already handed off to us; clear the AppCpu boot address so a subsequent software
    // reset doesn't re-enter this stale entry point. Matches IDF's `call_start_cpu1`.
    crate::rom::ets_set_appcpu_boot_addr(0);

    unsafe {
        #[cfg(all(feature = "rt", stack_guard_monitoring))]
        {
            let guard = APP_CORE_STACK_GUARD.load(Ordering::Acquire);
            guard.write_volatile(esp_config::esp_config_int!(
                u32,
                "ESP_HAL_CONFIG_STACK_GUARD_VALUE"
            ));
            crate::debugger::set_stack_watchpoint(guard as usize);
        }
        crate::interrupt::init_vectoring();
        #[cfg(feature = "rt")]
        crate::interrupt::ipc::install_app();
    }

    unsafe { CpuControl::start_core1_run::<F>() }
}
