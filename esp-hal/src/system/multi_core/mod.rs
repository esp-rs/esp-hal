//! Multi-core support

use core::{
    marker::PhantomData,
    mem::{ManuallyDrop, MaybeUninit},
    sync::atomic::{AtomicPtr, Ordering},
};

use esp_sync::{
    RestoreState,
    raw::{RawLock, SingleCoreInterruptLock},
};

use crate::{peripherals::CPU_CTRL, system::Cpu};

/// The park symbols use this value for "the other core" and for "no core".
const NO_CORE: u8 = u8::MAX;

fn cpu_from_u8(core: u8) -> Cpu {
    debug_assert!((core as usize) < Cpu::COUNT);
    if core == 0 { Cpu::ProCpu } else { Cpu::AppCpu }
}

// Runtime contract, keep stable.
cfg_select! {
    feature = "rt" => {
        pub(crate) mod park;
        mod start;

        pub(crate) use park::unpark;
        use park::{park_request, park_stall, stop_app_core};
        pub(crate) use start::app_core_takes_ipc;
        use start::{APP_CORE_STOPPING, APP_CORE_TAKES_IPC, app_core_running, start_app_core};

        use crate::interrupt::ipc::{defer_context_switch, resume_context_switch};
    }
    _ => {
        // The esp-hal copy with the `rt` feature defines these functions in `park` and `start`.
        unsafe extern "C" {
            fn esp_hal_park_request(core: u8, generation: &mut u32) -> u8;
            fn esp_hal_park_stall(core: u8, generation: u32) -> bool;
            fn esp_hal_unpark_core(core: u8);
            fn esp_hal_stop_app_core();
        }

        unsafe extern "Rust" {
            #[link_name = "esp_hal_start_app_core"]
            fn start_app_core(
                stack_top: *mut u32,
                stack_guard: *mut u32,
                entry: unsafe fn(),
            ) -> bool;
            #[link_name = "esp_hal_app_core_running"]
            safe fn app_core_running() -> bool;
            #[link_name = "esp_hal_defer_context_switch"]
            safe fn defer_context_switch();
            #[link_name = "esp_hal_resume_context_switch"]
            fn resume_context_switch();
        }

        #[inline(always)]
        fn park_request(core: Option<Cpu>) -> Option<(Cpu, u32)> {
            let mut generation = 0;
            let parked = unsafe {
                esp_hal_park_request(core.map_or(NO_CORE, |core| core as u8), &mut generation)
            };
            (parked != NO_CORE).then(|| (cpu_from_u8(parked), generation))
        }

        #[inline(always)]
        fn park_stall(core: Cpu, generation: u32) -> bool {
            unsafe { esp_hal_park_stall(core as u8, generation) }
        }

        #[inline(always)]
        pub(crate) fn unpark(core: Cpu) {
            unsafe { esp_hal_unpark_core(core as u8) }
        }

        #[inline(always)]
        fn stop_app_core() {
            unsafe { esp_hal_stop_app_core() }
        }
    }
}

/// Parks `core`, and returns whether the call took a park that [`unpark`] must release.
///
/// Returns `false` if `core` runs no code of this program. Such a core has nothing to park.
///
/// # Panics
///
/// Panics if `core` is the current core.
#[crate::ram]
pub(crate) fn park(core: Cpu) -> bool {
    park_masked(core)
        .map(|token| unsafe { SingleCoreInterruptLock.exit(token) })
        .is_some()
}

/// Parks `core` as [`park`] does, but returns with the interrupts of the current core masked.
///
/// Returns the mask if the call took a park, or `None` if [`park`] would return `false`. The wait
/// for the park does not mask interrupts. A caller that calls [`unpark`] before it ends the mask
/// runs no interrupt handler while `core` is parked.
///
/// # Panics
///
/// Panics if `core` is the current core.
#[crate::ram]
pub(crate) fn park_masked(core: Cpu) -> Option<RestoreState> {
    park_target_masked(Some(core)).map(|(_, token)| token)
}

/// Parks the other core as [`park_masked`] does, and returns the parked core with the mask.
#[crate::ram]
pub(crate) fn park_other_masked() -> Option<(Cpu, RestoreState)> {
    park_target_masked(None)
}

#[inline(always)]
fn park_target_masked(core: Option<Cpu>) -> Option<(Cpu, RestoreState)> {
    loop {
        let (parked, generation) = park_request(core)?;
        // The mask belongs to this copy of esp-hal, so it never crosses the park symbols.
        let token = unsafe { SingleCoreInterruptLock.enter() };
        if park_stall(parked, generation) {
            return Some((parked, token));
        }
        unsafe { SingleCoreInterruptLock.exit(token) };
    }
}

/// Returns whether `core` runs code of this program. Only such a core takes park requests.
#[cfg(feature = "rt")]
#[inline(always)]
fn takes_ipc(core: Cpu) -> bool {
    core == Cpu::ProCpu || app_core_takes_ipc()
}

/// Returns whether `core` runs code of this program.
///
/// The APP core runs code of this program after [`CpuControl`] started it, until it is parked or
/// stopped.
#[instability::unstable]
pub fn is_running(core: Cpu) -> bool {
    match core {
        Cpu::ProCpu => crate::soc::cpu_control::is_running(core),
        Cpu::AppCpu => app_core_running(),
    }
}

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

/// Stops the APP (second) core when dropped.
///
/// The drop parks the APP core as [`CpuControl::park_core`] does, and waits in the same way. If the
/// APP core never takes the IPC interrupt, the drop waits forever. The APP core then stays parked,
/// also after the last [`ParkGuard`] drops.
/// [`CpuControl::start_app_core`] can start the APP core again.
#[must_use = "Dropping this guard will stop the APP core"]
#[instability::unstable]
pub struct AppCoreGuard<'a> {
    phantom: PhantomData<&'a ()>,
}

impl Drop for AppCoreGuard<'_> {
    fn drop(&mut self) {
        stop_app_core();
    }
}

/// Keeps a core parked. The core runs again when the last guard for the core drops.
///
/// [`CpuControl::park_core`] returns this guard.
#[must_use = "Dropping this guard unparks the core"]
#[instability::unstable]
pub struct ParkGuard<'a> {
    core: Option<Cpu>,
    // The guard must drop on the core that parked, because only that core runs.
    phantom: PhantomData<(&'a (), *const ())>,
}

impl ParkGuard<'_> {
    /// Drops the guard without unparking the core.
    ///
    /// [`CpuControl::start_app_core`] can start a parked APP core again. No function can start a
    /// parked PRO core again.
    #[instability::unstable]
    pub fn keep_parked(self) {
        core::mem::forget(self);
    }
}

impl Drop for ParkGuard<'_> {
    fn drop(&mut self) {
        if let Some(core) = self.core {
            unpark(core);
        }
    }
}

/// Suspends the scheduler of the current core until the guard drops.
///
/// The current core defers its context switches while a guard is alive. The current thread thus
/// keeps the current core, and cannot move to the other core. Interrupt handlers and
/// [IPC requests][crate::interrupt::ipc::IpcRequest] still run. The deferred context switch runs
/// when the last guard drops.
///
/// The scheduler of the other core continues to run. Do not keep a guard for a long time,
/// because other threads cannot run on the current core.
#[must_use = "The scheduler continues when the guard drops"]
#[instability::unstable]
pub struct SchedulerSuspendGuard {
    // The guard must drop on the core that made the guard.
    _not_send: PhantomData<*const ()>,
}

impl SchedulerSuspendGuard {
    /// Creates a new guard, and suspends the scheduler of the current core.
    #[instability::unstable]
    #[inline(always)]
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        defer_context_switch();
        Self {
            _not_send: PhantomData,
        }
    }
}

impl Drop for SchedulerSuspendGuard {
    #[inline(always)]
    fn drop(&mut self) {
        unsafe { resume_context_switch() };
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

    /// Parks `core` until the returned guard drops.
    ///
    /// `core` parks in the IPC interrupt, which has the lowest priority. `core` thus holds no lock
    /// that masks interrupts when it stops. `core` can hold a lock that leaves interrupts on, for
    /// example a spin lock in thread mode. Do not take such a lock while `core` is parked. The
    /// parked core keeps its interrupts masked. The parked core thus handles no interrupt, also not
    /// the tick of a scheduler. [IPC requests][crate::interrupt::ipc::IpcRequest] to the parked
    /// core run after the park ends.
    ///
    /// The call waits until `core` parks. The wait does not mask interrupts, so interrupt
    /// handlers can run, and can park `core` too. The current core defers its context switches
    /// during the wait. If `core` never takes the IPC interrupt, the
    /// call waits forever. For example, `core` can keep its interrupts masked, or wait for a lock
    /// in an interrupt handler. Do not hold a lock that `core` can wait for in an interrupt handler
    /// or with masked interrupts, for example a critical section or a mutex.
    ///
    /// Parks add up. `core` runs again when the last [`ParkGuard`] for `core` drops. If `core` runs
    /// no code of this program, the call does nothing. See [`is_running`].
    ///
    /// A parked core can wait while the other core suspends the caches. See [Optimization
    /// level](crate#optimization-level) for the build requirement.
    ///
    /// # Panics
    ///
    /// Panics if `core` is the current core. A thread that an RTOS can move between cores can run
    /// on `core` at the call.
    #[instability::unstable]
    pub fn park_core(&self, core: Cpu) -> ParkGuard<'_> {
        ParkGuard {
            core: park(core).then_some(core),
            phantom: PhantomData,
        }
    }

    /// Parks the other core until the returned guard drops, as [`Self::park_core`] does.
    ///
    /// The call chooses the other core while the current thread cannot move to it. A thread that an
    /// RTOS can move between cores can thus call this function.
    #[instability::unstable]
    pub fn park_other_core(&self) -> ParkGuard<'_> {
        ParkGuard {
            core: park_other_masked().map(|(core, token)| {
                unsafe { SingleCoreInterruptLock.exit(token) };
                core
            }),
            phantom: PhantomData,
        }
    }

    /// Starts the APP (second) core.
    ///
    /// The second core starts running the closure `entry`. The core enables interrupts before
    /// `entry` runs. If the closure returns, the core stops.
    ///
    /// Dropping the returned guard stops the core.
    ///
    /// The APP core can park while the other core suspends the caches. See [Optimization
    /// level](crate#optimization-level) for the build requirement.
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
    /// The second core starts running the closure `entry`. The core enables interrupts before
    /// `entry` runs. If the closure returns, the core stops.
    ///
    /// Dropping the returned guard stops the core.
    ///
    /// The APP core can park while the other core suspends the caches. See [Optimization
    /// level](crate#optimization-level) for the build requirement.
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
        let (entry, stack_top, stack_guard) =
            setup_second_core_stack(stack, stack_guard_offset, entry);

        APP_CORE_CLOSURE.store(entry, Ordering::Release);
        if !unsafe { start_app_core(stack_top, stack_guard, run_entry::<F>) } {
            APP_CORE_CLOSURE.store(core::ptr::null_mut(), Ordering::Relaxed);
            unsafe { ManuallyDrop::drop(&mut *entry.cast::<ManuallyDrop<F>>()) };
            return Err(Error::CoreAlreadyRunning);
        }

        // A later call stores its closure in `APP_CORE_CLOSURE` also when it fails.
        while !APP_CORE_CLOSURE.load(Ordering::Acquire).is_null() {
            core::hint::spin_loop();
        }

        Ok(AppCoreGuard {
            phantom: PhantomData,
        })
    }
}

static APP_CORE_CLOSURE: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

/// Calls the closure that [`setup_second_core_stack`] placed on the stack.
unsafe fn run_entry<F>()
where
    F: FnOnce(),
{
    let entry = APP_CORE_CLOSURE.swap(core::ptr::null_mut(), Ordering::AcqRel);
    unsafe { ManuallyDrop::take(&mut *entry.cast::<ManuallyDrop<F>>())() }
}

/// Places `entry` on the stack. Returns the address of `entry`, the stack top and the stack guard.
fn setup_second_core_stack<'a, F, const SIZE: usize>(
    stack: &'static mut Stack<SIZE>,
    stack_guard_offset: Option<usize>,
    entry: F,
) -> (*mut (), *mut u32, *mut u32)
where
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

        (entry_dst.cast(), stack.top(), stack_guard.cast())
    }
}
