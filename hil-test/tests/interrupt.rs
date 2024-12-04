//! Interrupt Test
//!
//! "Disabled" for now - see https://github.com/esp-rs/esp-hal/pull/1635#issuecomment-2137405251

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use core::{arch::asm, cell::RefCell};

use critical_section::Mutex;
use esp_hal::{
    interrupt::{
        self,
        software::{SoftwareInterrupt, SoftwareInterruptControl},
        CpuInterrupt,
        Priority,
    },
    peripherals::Interrupt,
    prelude::*,
};
use hil_test as _;

static SWINT0: Mutex<RefCell<Option<SoftwareInterrupt<0>>>> = Mutex::new(RefCell::new(None));

#[allow(unused)] // TODO: Remove attribute when interrupt latency test re-enabled
struct Context {
    sw0_trigger_addr: u32,
}

#[no_mangle]
fn interrupt20() {
    unsafe { asm!("csrrwi x0, 0x7e1, 0 #disable timer") }
    critical_section::with(|cs| {
        SWINT0.borrow_ref(cs).as_ref().unwrap().reset();
    });

    let mut perf_counter: u32 = 0;
    unsafe {
        asm!(
            "
            csrr {x}, 0x7e2
            ",
            options(nostack),
            x = inout(reg) perf_counter,
        )
    };
    defmt::info!("Performance counter:{}", perf_counter);
    // TODO these values should be adjusted to catch smaller regressions
    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32c3", feature = "esp32c2"))] {
            assert!(perf_counter < 1100);
        } else {
            assert!(perf_counter < 750);
        }
    }
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let peripherals = esp_hal::init(config);
        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32c6", feature = "esp32h2"))] {
                let cpu_intr = &peripherals.INTPRI;
            } else {
                let cpu_intr = &peripherals.SYSTEM;
            }
        }

        let sw0_trigger_addr = cpu_intr.cpu_intr_from_cpu_0() as *const _ as u32;

        critical_section::with(|cs| {
            SWINT0
                .borrow_ref_mut(cs)
                .replace(sw_ints.software_interrupt0)
        });
        interrupt::enable_direct(
            Interrupt::FROM_CPU_INTR0,
            Priority::Priority3,
            CpuInterrupt::Interrupt20,
        )
        .unwrap();

        Context { sw0_trigger_addr }
    }

    #[test]
    #[rustfmt::skip]
    fn interrupt_latency(_ctx: Context) {
        // unsafe {
        //     asm!(
        //         "
        // csrrwi x0, 0x7e0, 1 #what to count, for cycles write 1 for instructions write 2
        // csrrwi x0, 0x7e1, 0 #disable counter
        // csrrwi x0, 0x7e2, 0 #reset counter
        // "
        //     );
        // }

        // // interrupt is raised from assembly for max timer granularity.
        // unsafe {
        //     asm!(
        //         "
        // li {bit}, 1                   # Flip flag (bit 0)
        // csrrwi x0, 0x7e1, 1           # enable timer
        // sw {bit}, 0({addr})           # trigger FROM_CPU_INTR0
        // ",
        //     options(nostack),
        //     addr = in(reg) ctx.sw0_trigger_addr,
        //     bit = out(reg) _,
        //     )
        // }
    }
}
