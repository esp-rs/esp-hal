//! Control CPU Cores

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
    pub fn start_app_core(
        &mut self,
        entry: &mut (dyn FnMut() + Send),
    ) -> Result<AppCoreGuard, Error> {
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
