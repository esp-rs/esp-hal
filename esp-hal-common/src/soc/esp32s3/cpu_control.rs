//! Control CPU Cores(ESP32-S3)
//!
//! ## Overview
//!
//! This module is part of the `SOC (System-on-Chip)` functionality of the
//! `ESP32-S3` chip. It module provides essential functionality for controlling
//! and managing the CPU cores on the `ESP32-S3` chip allowing for fine-grained
//! control over their execution and cache behavior. It is used in scenarios
//! where precise control over CPU core operation is required, such as
//! multi-threading or power management.
//!
//! The `CpuControl` struct represents the CPU control module and is responsible
//! for managing the behavior and operation of the CPU cores. It is typically
//! initialized with the `SystemCpuControl` struct, which is provided by the
//! `system` module.
//!
//! ## Example
//! ```no_run
//! let counter = Mutex::new(RefCell::new(0));
//!
//! let mut cpu_control = CpuControl::new(system.cpu_control);
//! let mut cpu1_fnctn = || {
//!     cpu1_task(&mut timer1, &counter);
//! };
//! let _guard = cpu_control.start_app_core(&mut cpu1_fnctn).unwrap();
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

use core::marker::PhantomData;

use xtensa_lx::set_stack_pointer;

use crate::Cpu;

static mut START_CORE1_FUNCTION: Option<&'static mut (dyn FnMut() + 'static)> = None;

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
        let rtc_control = crate::peripherals::RTC_CNTL::PTR;
        let rtc_control = unsafe { &*rtc_control };

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

    unsafe fn start_core1_init() -> ! {
        extern "C" {
            static mut _stack_end_cpu1: u32;
        }

        // disables interrupts
        xtensa_lx::interrupt::set_mask(0);

        // reset cycle compare registers
        xtensa_lx::timer::set_ccompare0(0);
        xtensa_lx::timer::set_ccompare1(0);
        xtensa_lx::timer::set_ccompare2(0);

        // set stack pointer to end of memory: no need to retain stack up to this point
        set_stack_pointer(&mut _stack_end_cpu1);

        extern "C" {
            static mut _init_start: u32;
        }

        unsafe {
            // move vec table
            let base = &_init_start as *const u32;
            core::arch::asm!("wsr.vecbase {0}", in(reg) base, options(nostack));
        }

        match START_CORE1_FUNCTION.take() {
            Some(entry) => (*entry)(),
            None => panic!("No start function set"),
        }

        panic!("Return from second core's entry");
    }

    /// Start the APP (second) core
    ///
    /// The second core will start running the closure `entry`.
    ///
    /// Dropping the returned guard will park the core.
    pub fn start_app_core<'a>(
        &mut self,
        entry: &'a mut (dyn FnMut() + Send),
    ) -> Result<AppCoreGuard<'a>, Error> {
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

        unsafe {
            let entry_fn: &'static mut (dyn FnMut() + 'static) = core::mem::transmute(entry);
            START_CORE1_FUNCTION = Some(entry_fn);
        }

        // TODO there is no boot_addr register in SVD or TRM - ESP-IDF uses a ROM
        // function so we also have to for now
        const ETS_SET_APPCPU_BOOT_ADDR: usize = 0x40000720;
        unsafe {
            let ets_set_appcpu_boot_addr: unsafe extern "C" fn(u32) =
                core::mem::transmute(ETS_SET_APPCPU_BOOT_ADDR);
            ets_set_appcpu_boot_addr(Self::start_core1_init as *const u32 as u32);
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
