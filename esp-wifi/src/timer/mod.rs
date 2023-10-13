#[cfg_attr(esp32, path = "timer_esp32.rs")]
#[cfg_attr(esp32c2, path = "timer_esp32c2.rs")]
#[cfg_attr(esp32c3, path = "timer_esp32c3.rs")]
#[cfg_attr(esp32c6, path = "timer_esp32c6.rs")]
#[cfg_attr(esp32s3, path = "timer_esp32s3.rs")]
#[cfg_attr(esp32s2, path = "timer_esp32s2.rs")]
mod chip_specific;

#[cfg_attr(any(esp32, esp32s2, esp32s3), path = "xtensa.rs")]
#[cfg_attr(any(esp32c2, esp32c3, esp32c6), path = "riscv.rs")]
mod arch_specific;

pub use arch_specific::*;
pub use chip_specific::*;

pub fn setup_timer_isr(timebase: TimeBase) {
    setup_radio_isr();

    setup_timer(timebase);

    setup_multitasking();
}
