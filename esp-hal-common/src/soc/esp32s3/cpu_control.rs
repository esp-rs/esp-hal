//! # Control CPU Cores (ESP32-S3)
//!
//! ## Overview
//!
//! This module provides essential functionality for controlling
//! and managing the APP (second) CPU core on the `ESP32-S3` chip. It is used to
//! start and stop program execution on the APP core.
//!
//! ## Example
//!
//! ```no_run
//! static mut APP_CORE_STACK: Stack<8192> = Stack::new();
//!
//! let counter = Mutex::new(RefCell::new(0));
//!
//! let mut cpu_control = CpuControl::new(system.cpu_control);
//! let cpu1_fnctn = || {
//!     cpu1_task(&mut timer1, &counter);
//! };
//! let _guard = cpu_control
//!     .start_app_core(unsafe { &mut APP_CORE_STACK }, cpu1_fnctn)
//!     .unwrap();
//!
//! loop {
//!     block!(timer0.wait()).unwrap();
//!
//!     let count = critical_section::with(|cs| *counter.borrow_ref(cs));
//!     println!("Hello World - Core 0! Counter is {}", count);
//! }
//! ```
//!
//! Where `cpu1_task()` may be defined as:
//!
//! ```no_run
//! fn cpu1_task(
//!     timer: &mut Timer<Timer0<TIMG1>>,
//!     counter: &critical_section::Mutex<RefCell<i32>>,
//! ) -> ! {
//!     println!("Hello World - Core 1!");
//!     loop {
//!         block!(timer.wait()).unwrap();
//!
//!         critical_section::with(|cs| {
//!             let new_val = counter.borrow_ref_mut(cs).wrapping_add(1);
//!             *counter.borrow_ref_mut(cs) = new_val;
//!         });
//!     }
//! }
//! ```

use core::{
    marker::PhantomData,
    mem::{ManuallyDrop, MaybeUninit},
};

use xtensa_lx::set_stack_pointer;

use crate::Cpu;

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
pub struct Stack<const SIZE: usize> {
    /// Memory to be used for the stack
    pub mem: MaybeUninit<[u8; SIZE]>,
}

impl<const SIZE: usize> Stack<SIZE> {
    const _ALIGNED: () = assert!(SIZE % 16 == 0); // Make sure stack top is aligned, too.

    /// Construct a stack of length SIZE, uninitialized
    #[allow(path_statements)]
    pub const fn new() -> Stack<SIZE> {
        Self::_ALIGNED;

        Stack {
            mem: MaybeUninit::uninit(),
        }
    }

    pub const fn len(&self) -> usize {
        SIZE
    }

    pub fn bottom(&mut self) -> *mut u32 {
        self.mem.as_mut_ptr() as *mut u32
    }

    pub fn top(&mut self) -> *mut u32 {
        unsafe { self.bottom().add(SIZE) }
    }
}

// Pointer to the closure that will be executed on the second core. The closure
// is copied to the core's stack.
static mut START_CORE1_FUNCTION: Option<*mut ()> = None;

static mut APP_CORE_STACK_TOP: Option<*mut u32> = None;

/// Will park the APP (second) core when dropped
#[must_use]
pub struct AppCoreGuard<'a> {
    phantom: PhantomData<&'a ()>,
}

impl<'a> Drop for AppCoreGuard<'a> {
    fn drop(&mut self) {
        unsafe {
            internal_park_core(Cpu::AppCpu);
        }
    }
}

#[derive(Debug)]
pub enum Error {
    CoreAlreadyRunning,
}

/// Control CPU Cores
pub struct CpuControl {
    _cpu_control: crate::system::CpuControl,
}

unsafe fn internal_park_core(core: Cpu) {
    let rtc_control = crate::peripherals::RTC_CNTL::PTR;
    let rtc_control = &*rtc_control;

    match core {
        Cpu::ProCpu => {
            rtc_control
                .sw_cpu_stall
                .modify(|_, w| w.sw_stall_procpu_c1().bits(0x21));
            rtc_control
                .options0
                .modify(|_, w| w.sw_stall_procpu_c0().bits(0x02));
        }
        Cpu::AppCpu => {
            rtc_control
                .sw_cpu_stall
                .modify(|_, w| w.sw_stall_appcpu_c1().bits(0x21));
            rtc_control
                .options0
                .modify(|_, w| w.sw_stall_appcpu_c0().bits(0x02));
        }
    }
}

impl CpuControl {
    pub fn new(cpu_control: crate::system::CpuControl) -> CpuControl {
        CpuControl {
            _cpu_control: cpu_control,
        }
    }

    /// Park the given core
    pub unsafe fn park_core(&mut self, core: Cpu) {
        internal_park_core(core);
    }

    /// Unpark the given core
    pub fn unpark_core(&mut self, core: Cpu) {
        let rtc_control = unsafe { &*crate::peripherals::RTC_CNTL::ptr() };

        match core {
            Cpu::ProCpu => {
                rtc_control
                    .sw_cpu_stall
                    .modify(|_, w| unsafe { w.sw_stall_procpu_c1().bits(0) });
                rtc_control
                    .options0
                    .modify(|_, w| unsafe { w.sw_stall_procpu_c0().bits(0) });
            }
            Cpu::AppCpu => {
                rtc_control
                    .sw_cpu_stall
                    .modify(|_, w| unsafe { w.sw_stall_appcpu_c1().bits(0) });
                rtc_control
                    .options0
                    .modify(|_, w| unsafe { w.sw_stall_appcpu_c0().bits(0) });
            }
        }
    }

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

        // set stack pointer to end of memory: no need to retain stack up to this point
        set_stack_pointer(unsafe { APP_CORE_STACK_TOP.unwrap() });

        extern "C" {
            static mut _init_start: u32;
        }

        unsafe {
            // move vec table
            let base = &_init_start as *const u32;
            core::arch::asm!("wsr.vecbase {0}", in(reg) base, options(nostack));
        }

        match START_CORE1_FUNCTION.take() {
            Some(entry) => {
                let entry = unsafe { ManuallyDrop::take(&mut *entry.cast::<ManuallyDrop<F>>()) };
                entry();
                loop {
                    unsafe { internal_park_core(crate::get_core()) };
                }
            }
            None => panic!("No start function set"),
        }
    }

    /// Start the APP (second) core
    ///
    /// The second core will start running the closure `entry`.
    ///
    /// Dropping the returned guard will park the core.
    pub fn start_app_core<'a, const SIZE: usize, F>(
        &mut self,
        stack: &'static mut Stack<SIZE>,
        entry: F,
    ) -> Result<AppCoreGuard<'a>, Error>
    where
        F: FnOnce(),
        F: Send + 'a,
    {
        let system_control = crate::peripherals::SYSTEM::PTR;
        let system_control = unsafe { &*system_control };

        if !xtensa_lx::is_debugger_attached()
            && system_control
                .core_1_control_0
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

        // TODO there is no boot_addr register in SVD or TRM - ESP-IDF uses a ROM
        // function so we also have to for now
        const ETS_SET_APPCPU_BOOT_ADDR: usize = 0x40000720;
        unsafe {
            let ets_set_appcpu_boot_addr: unsafe extern "C" fn(u32) =
                core::mem::transmute(ETS_SET_APPCPU_BOOT_ADDR);
            ets_set_appcpu_boot_addr(Self::start_core1_init::<F> as *const u32 as u32);
        };

        system_control
            .core_1_control_0
            .modify(|_, w| w.control_core_1_clkgate_en().set_bit());
        system_control
            .core_1_control_0
            .modify(|_, w| w.control_core_1_runstall().clear_bit());
        system_control
            .core_1_control_0
            .modify(|_, w| w.control_core_1_reseting().set_bit());
        system_control
            .core_1_control_0
            .modify(|_, w| w.control_core_1_reseting().clear_bit());

        self.unpark_core(Cpu::AppCpu);

        Ok(AppCoreGuard {
            phantom: PhantomData::default(),
        })
    }
}
