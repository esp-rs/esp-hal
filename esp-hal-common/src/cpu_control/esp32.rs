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

    fn flush_cache(&mut self, core: Cpu) {
        let dport_control = crate::peripherals::DPORT::PTR;
        let dport_control = unsafe { &*dport_control };

        match core {
            Cpu::ProCpu => {
                dport_control
                    .pro_cache_ctrl
                    .modify(|_, w| w.pro_cache_flush_ena().clear_bit());
                dport_control
                    .pro_cache_ctrl
                    .modify(|_, w| w.pro_cache_flush_ena().set_bit());
                while dport_control
                    .pro_cache_ctrl
                    .read()
                    .pro_cache_flush_done()
                    .bit_is_clear()
                {}

                dport_control
                    .pro_cache_ctrl
                    .modify(|_, w| w.pro_cache_flush_ena().clear_bit());
            }
            Cpu::AppCpu => {
                dport_control
                    .app_cache_ctrl
                    .modify(|_, w| w.app_cache_flush_ena().clear_bit());
                dport_control
                    .app_cache_ctrl
                    .modify(|_, w| w.app_cache_flush_ena().set_bit());
                while dport_control
                    .app_cache_ctrl
                    .read()
                    .app_cache_flush_done()
                    .bit_is_clear()
                {}
                dport_control
                    .app_cache_ctrl
                    .modify(|_, w| w.app_cache_flush_ena().clear_bit());
            }
        };
    }

    fn enable_cache(&mut self, core: Cpu) {
        let spi0 = unsafe { &(*crate::peripherals::SPI0::ptr()) };

        let dport_control = crate::peripherals::DPORT::PTR;
        let dport_control = unsafe { &*dport_control };

        match core {
            Cpu::ProCpu => {
                spi0.cache_fctrl.modify(|_, w| w.cache_req_en().set_bit());
                dport_control
                    .pro_cache_ctrl
                    .modify(|_, w| w.pro_cache_enable().set_bit());
            }
            Cpu::AppCpu => {
                spi0.cache_fctrl.modify(|_, w| w.cache_req_en().set_bit());
                dport_control
                    .app_cache_ctrl
                    .modify(|_, w| w.app_cache_enable().set_bit());
            }
        };
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
        let dport_control = crate::peripherals::DPORT::PTR;
        let dport_control = unsafe { &*dport_control };

        if !xtensa_lx::is_debugger_attached()
            && dport_control
                .appcpu_ctrl_b
                .read()
                .appcpu_clkgate_en()
                .bit_is_set()
        {
            return Err(Error::CoreAlreadyRunning);
        }

        self.flush_cache(Cpu::AppCpu);
        self.enable_cache(Cpu::AppCpu);

        unsafe {
            let entry_fn: &'static mut (dyn FnMut() + 'static) = core::mem::transmute(entry);
            START_CORE1_FUNCTION = Some(entry_fn);
        }

        dport_control.appcpu_ctrl_d.write(|w| unsafe {
            w.appcpu_boot_addr()
                .bits(Self::start_core1_init as *const u32 as u32)
        });

        dport_control
            .appcpu_ctrl_b
            .modify(|_, w| w.appcpu_clkgate_en().set_bit());
        dport_control
            .appcpu_ctrl_c
            .modify(|_, w| w.appcpu_runstall().clear_bit());
        dport_control
            .appcpu_ctrl_a
            .modify(|_, w| w.appcpu_resetting().set_bit());
        dport_control
            .appcpu_ctrl_a
            .modify(|_, w| w.appcpu_resetting().clear_bit());

        self.unpark_core(Cpu::AppCpu);

        Ok(AppCoreGuard {
            phantom: PhantomData::default(),
        })
    }
}
