//! Interrupt Test
//!
//! "Disabled" for now - see https://github.com/esp-rs/esp-hal/pull/1635#issuecomment-2137405251

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2
//% FEATURES: unstable

#![no_std]
#![no_main]

use core::{arch::asm, cell::RefCell};

use critical_section::Mutex;
use esp_hal::{
    clock::CpuClock,
    interrupt::{
        self,
        CpuInterrupt,
        Priority,
        software::{SoftwareInterrupt, SoftwareInterruptControl},
    },
    peripherals::Interrupt,
};
use hil_test as _;

static SWINT0: Mutex<RefCell<Option<SoftwareInterrupt<0>>>> = Mutex::new(RefCell::new(None));

#[unsafe(no_mangle)]
static mut LAST_PERF: u32 = 0;

#[unsafe(no_mangle)]
static mut SW_TRIGGER_ADDR: *mut u32 = core::ptr::null_mut();

struct Context {
    sw0_trigger_addr: u32,
}

// We need to place this close to the trap handler for the jump to be resolved properly
#[unsafe(link_section = ".trap.rust")]
#[unsafe(no_mangle)]
#[unsafe(naked)]
#[rustfmt::skip]
unsafe extern "C" fn interrupt_handler() {
    core::arch::naked_asm! {"
        # save affected registers
        addi sp, sp, -16*4 # allocate 16 words for saving regs (will work with just 2, but RISC-V wants it to be aligned by 16)
        sw t0, 1*4(sp)
        sw t1, 2*4(sp)

        csrrwi x0, 0x7e1, 0 #disable timer

        lw t0, {sw}  # t0 <- &SW_TRIGGER_ADDR
        sw x0, 0(t0) # clear (deassert) *SW_TRIGGER_ADDR

        csrr t1, 0x7e2 # read performance counter
        la t0, {x}
        sw t1, 0(t0) # store word to LAST_PERF

        # restore affected registers
        lw t0, 1*4(sp)
        lw t1, 2*4(sp)
        addi sp, sp, 16*4
        mret
        ",
        x = sym LAST_PERF,
        sw = sym SW_TRIGGER_ADDR,
    }
}

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

        let sw0_trigger_addr = cpu_intr.register_block().cpu_intr_from_cpu(0) as *const _ as u32;
        unsafe {
            SW_TRIGGER_ADDR = sw0_trigger_addr as *mut u32;
        }

        critical_section::with(|cs| {
            SWINT0
                .borrow_ref_mut(cs)
                .replace(sw_ints.software_interrupt0)
        });

        interrupt::enable_direct(
            Interrupt::FROM_CPU_INTR0,
            Priority::Priority3,
            CpuInterrupt::Interrupt20,
            interrupt_handler,
        )
        .unwrap();

        Context { sw0_trigger_addr }
    }

    // Handler function (assigned above) reads cycles count from CSR and stores it into `LAST_PERF`
    // static, which this test then reads and asserts its value to detect regressions
    #[test]
    #[rustfmt::skip]
    fn interrupt_latency(ctx: Context) {
        unsafe {
            asm!(
                "
                    csrrwi x0, 0x7e0, 1 #what to count, for cycles write 1 for instructions write 2
                    csrrwi x0, 0x7e1, 0 #disable counter
                    csrrwi x0, 0x7e2, 0 #reset counter
                "
            );
        }
        
        // interrupt is raised from assembly for max timer granularity.
        unsafe {
            asm!(
                "
                    li {bit}, 1                   # Flip flag (bit 0)
                    csrrwi x0, 0x7e1, 1           # enable timer
                    sw {bit}, 0({addr})           # trigger FROM_CPU_INTR0
                ",
                options(nostack),
                addr = in(reg) ctx.sw0_trigger_addr,
                bit = out(reg) _,
            )
        }

        let perf_counter = unsafe { LAST_PERF };
        defmt::info!("Performance counter: {}", perf_counter);

        // TODO c3/c2 values should be adjusted to catch smaller regressions
        cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32c3", feature = "esp32c2"))] {
            assert!(perf_counter < 400);
        } else {
            assert!(perf_counter < 155);
        }
    }
    }
}
