//! Control the ULP RISCV core
//! 
//! ## Overview
//! 
//! The `ULP RISCV CORE` peripheral allows control over the `Ultra-Low Power (ULP) RISC-V core` in `ESP` chips.
//! The ULP core is a low-power processor designed for executing tasks in deep sleep mode, enabling
//! efficient power management in ESP systems.
//! 
//! The `UlpCore` struct provides an interface to interact with the `ULP RISCV CORE` peripheral. 
//! It allows starting and configuring the ULP core for operation.
//! The `UlpCore` struct is initialized with a peripheral reference to the `ULP RISCV CORE` instance.
//! 
//! ## Example
//! ```no_run
//! let mut ulp_core = esp32s3_hal::ulp_core::UlpCore::new(peripherals.ULP_RISCV_CORE);
//! ulp_core.stop();
//! println!("ulp core stopped");
//!
//! // copy code to RTC ram
//! let lp_ram = 0x5000_0000 as *mut u8;
//! unsafe {
//!     core::ptr::copy_nonoverlapping(CODE as *const _ as *const u8, lp_ram, CODE.len());
//! }
//! println!("copied code (len {})", CODE.len());
//!
//! // start ULP core
//! ulp_core.run(esp32s3_hal::ulp_core::UlpCoreWakeupSource::HpCpu);
//! println!("ulpcore run");
//!
//! let data = (0x5000_0010 - 0) as *mut u32;
//! loop {
//!     println!("Current {}", unsafe { data.read_volatile() });
//! }
//!
//! ```
use esp32s2 as pac;

use crate::peripheral::{Peripheral, PeripheralRef};

extern "C" {
    fn ets_delay_us(delay: u32);
}

#[derive(Debug, Clone, Copy)]
pub enum UlpCoreWakeupSource {
    HpCpu,
}

pub struct UlpCore<'d> {
    _lp_core: PeripheralRef<'d, crate::soc::peripherals::ULP_RISCV_CORE>,
}

impl<'d> UlpCore<'d> {
    pub fn new(lp_core: impl Peripheral<P = crate::soc::peripherals::ULP_RISCV_CORE> + 'd) -> Self {
        crate::into_ref!(lp_core);
        Self { _lp_core: lp_core }
    }

    // currently stopping the ULP doesn't work (while following the proсedures
    // outlines in the TRM) - so don't offer this funtion for now
    //
    // pub fn stop(&mut self) {
    //     ulp_stop();
    // }

    pub fn run(&mut self, wakeup_src: UlpCoreWakeupSource) {
        ulp_run(wakeup_src);
    }
}

#[allow(unused)] // TODO: remove cfg when implementation is corrected
fn ulp_stop() {
    let rtc_cntl = unsafe { &*pac::RTC_CNTL::PTR };
    rtc_cntl
        .ulp_cp_timer
        .modify(|_, w| w.ulp_cp_slp_timer_en().clear_bit());

    // suspends the ulp operation
    rtc_cntl.cocpu_ctrl.modify(|_, w| w.cocpu_done().set_bit());

    // Resets the processor
    rtc_cntl
        .cocpu_ctrl
        .modify(|_, w| w.cocpu_shut_reset_en().set_bit());
}

fn ulp_run(wakeup_src: UlpCoreWakeupSource) {
    let rtc_cntl = unsafe { &*pac::RTC_CNTL::PTR };

    // Reset COCPU when power on
    rtc_cntl
        .cocpu_ctrl
        .modify(|_, w| w.cocpu_shut_reset_en().set_bit());

    // Disable ULP timer
    rtc_cntl
        .ulp_cp_timer
        .modify(|_, w| w.ulp_cp_slp_timer_en().clear_bit());

    // wait for at least 1 RTC_SLOW_CLK cycle
    unsafe {
        ets_delay_us(20);
    }

    // Select ULP-RISC-V to send the DONE signal
    rtc_cntl
        .cocpu_ctrl
        .modify(|_, w| w.cocpu_done_force().set_bit());

    ulp_config_wakeup_source(wakeup_src);

    // Select RISC-V as the ULP_TIMER trigger target
    rtc_cntl.cocpu_ctrl.modify(|_, w| w.cocpu_sel().clear_bit());

    // Clear any spurious wakeup trigger interrupts upon ULP startup
    unsafe {
        ets_delay_us(20);
    }

    rtc_cntl.int_clr_rtc.write(|w| {
        w.cocpu_int_clr()
            .set_bit()
            .cocpu_trap_int_clr()
            .set_bit()
            .ulp_cp_int_clr()
            .set_bit()
    });
}

fn ulp_config_wakeup_source(wakeup_src: UlpCoreWakeupSource) {
    match wakeup_src {
        UlpCoreWakeupSource::HpCpu => {
            // use timer to wake up
            let rtc_cntl = unsafe { &*pac::RTC_CNTL::PTR };
            rtc_cntl
                .ulp_cp_ctrl
                .modify(|_, w| w.ulp_cp_force_start_top().clear_bit());
            rtc_cntl
                .ulp_cp_timer
                .modify(|_, w| w.ulp_cp_slp_timer_en().set_bit());
        }
    }
}
