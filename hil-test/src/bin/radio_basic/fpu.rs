//! Cp0Disable exception regression tests

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::peripherals::Peripherals;
    #[cfg(multi_core)]
    use esp_hal::system::CpuControl;

    #[inline(never)]
    fn run_float_calc(x: f32) -> f32 {
        let result = core::hint::black_box(x) * 2.0;
        defmt::info!("{}", defmt::Display2Format(&result));
        result
    }

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default();
        esp_hal::init(config)
    }

    #[test]
    fn fpu_is_enabled() {
        let result = run_float_calc(2.0);
        assert_eq!(result, 4.0);
    }

    #[cfg(multi_core)]
    #[test]
    fn fpu_is_enabled_on_core1(peripherals: Peripherals) {
        use core::sync::atomic::{AtomicBool, Ordering};

        static DONE: AtomicBool = AtomicBool::new(false);

        let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

        let guard = cpu_control
            .start_app_core(
                #[allow(static_mut_refs)]
                unsafe {
                    &mut crate::APP_CORE_STACK
                },
                || {
                    let result = run_float_calc(2.0);
                    assert_eq!(result, 4.0);
                    DONE.store(true, Ordering::Relaxed);
                    loop {}
                },
            )
            .unwrap();

        while !DONE.load(Ordering::Relaxed) {}
        let result = run_float_calc(2.0);
        assert_eq!(result, 4.0);

        core::mem::drop(guard);
    }

    #[cfg(multi_core)]
    #[test]
    fn fpu_is_enabled_on_core1_with_preempt(p: Peripherals) {
        use core::sync::atomic::{AtomicBool, Ordering};

        use esp_hal::{interrupt::software::SoftwareInterruptControl, timer::timg::TimerGroup};

        static DONE: AtomicBool = AtomicBool::new(false);

        let timg0 = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        esp_rtos::start_second_core::<8192>(
            p.CPU_CTRL,
            sw_ints.software_interrupt1,
            #[allow(static_mut_refs)]
            unsafe {
                &mut crate::APP_CORE_STACK
            },
            || {
                esp_radio_rtos_driver::usleep(10);

                let result = run_float_calc(2.0);
                assert_eq!(result, 4.0);
                DONE.store(true, Ordering::Relaxed);
                loop {}
            },
        );

        while !DONE.load(Ordering::Relaxed) {}
        let result = run_float_calc(2.0);
        assert_eq!(result, 4.0);
    }
}
