//! Cp0Disable exception regression test

//% CHIPS: esp32 esp32s2 esp32s3
//% FEATURES: unstable esp-radio esp-alloc esp-radio/wifi esp-radio/unstable

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    handler,
    interrupt::software::{SoftwareInterrupt, SoftwareInterruptControl},
    peripherals::Peripherals,
    timer::timg::TimerGroup,
};
use hil_test as _;

#[inline(never)]
fn run_float_calc(x: f32) -> f32 {
    let result = core::hint::black_box(x) * 2.0;
    defmt::info!("{}", defmt::Display2Format(&result));
    result
}

static SWINT0: Mutex<RefCell<Option<SoftwareInterrupt<0>>>> = Mutex::new(RefCell::new(None));

#[handler]
fn wait() {
    // Ensure esp-radio interrupts this handler.
    Delay::new().delay_millis(100);
    critical_section::with(|cs| {
        SWINT0.borrow_ref(cs).as_ref().unwrap().reset();
    });
}

cfg_if::cfg_if! {
    if #[cfg(multi_core)] {
        use core::sync::atomic::{AtomicBool, Ordering};

        use esp_hal::system::{CpuControl, Stack};

        static DONE: AtomicBool = AtomicBool::new(false);
        static mut APP_CORE_STACK: Stack<8192> = Stack::new();
    }
}

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn test_init() -> Peripherals {
        esp_alloc::heap_allocator!(size: 72 * 1024);

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    fn fpu_is_enabled() {
        let result = super::run_float_calc(2.0);
        assert_eq!(result, 4.0);
    }

    #[cfg(multi_core)]
    #[test]
    fn fpu_is_enabled_on_core1(peripherals: Peripherals) {
        let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

        let _guard = cpu_control
            .start_app_core(
                unsafe { &mut *core::ptr::addr_of_mut!(APP_CORE_STACK) },
                || {
                    let result = super::run_float_calc(2.0);
                    assert_eq!(result, 4.0);
                    DONE.store(true, Ordering::Relaxed);
                    loop {}
                },
            )
            .unwrap();

        core::mem::forget(_guard);

        while !DONE.load(Ordering::Relaxed) {}
        let result = super::run_float_calc(2.0);
        assert_eq!(result, 4.0);
    }

    #[test]
    fn fpu_stays_enabled_with_wifi(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_preempt::start(timg0.timer0);
        let _init = esp_radio::init().unwrap();

        let mut sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        critical_section::with(|cs| {
            sw_ints.software_interrupt0.set_interrupt_handler(wait);
            SWINT0
                .borrow_ref_mut(cs)
                .replace(sw_ints.software_interrupt0);
            // Fire a low-priority interrupt to ensure the FPU is disabled while
            // esp-radio switches tasks
            SWINT0.borrow_ref(cs).as_ref().unwrap().raise();
        });

        Delay::new().delay_millis(100);

        let result = super::run_float_calc(2.0);
        assert_eq!(result, 4.0);
    }

    #[cfg(multi_core)]
    #[test]
    fn fpu_stays_enabled_with_wifi_on_core1(peripherals: Peripherals) {
        let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

        let _guard = cpu_control
            .start_app_core(
                unsafe { &mut *core::ptr::addr_of_mut!(APP_CORE_STACK) },
                move || {
                    let timg0 = TimerGroup::new(peripherals.TIMG0);
                    esp_preempt::start(timg0.timer0);
                    let _init = esp_radio::init().unwrap();

                    let mut sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

                    critical_section::with(|cs| {
                        sw_ints.software_interrupt0.set_interrupt_handler(wait);
                        SWINT0
                            .borrow_ref_mut(cs)
                            .replace(sw_ints.software_interrupt0);
                        // Fire a low-priority interrupt to ensure the FPU is disabled while
                        // esp-radio switches tasks
                        SWINT0.borrow_ref(cs).as_ref().unwrap().raise();
                    });

                    Delay::new().delay_millis(100);

                    let result = super::run_float_calc(2.0);
                    assert_eq!(result, 4.0);
                    DONE.store(true, Ordering::Relaxed);
                    loop {}
                },
            )
            .unwrap();

        core::mem::forget(_guard);

        while !DONE.load(Ordering::Relaxed) {}

        let result = super::run_float_calc(2.0);
        assert_eq!(result, 4.0);
    }
}
