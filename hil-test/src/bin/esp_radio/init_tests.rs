#[embedded_test::tests(default_timeout = 3, executor = esp_rtos::embassy::Executor::new())]
mod init_tests {

    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
    #[cfg(riscv)]
    use esp_hal::riscv::interrupt::free as interrupt_free;
    #[cfg(xtensa)]
    use esp_hal::xtensa_lx::interrupt::free as interrupt_free;
    use esp_hal::{
        clock::CpuClock,
        interrupt::{
            Priority,
            software::{SoftwareInterrupt, SoftwareInterruptControl},
        },
        peripherals::{Peripherals, TIMG0},
        timer::timg::TimerGroup,
    };
    use esp_radio::InitializationError;
    use esp_rtos::embassy::InterruptExecutor;
    use hil_test::mk_static;
    use static_cell::StaticCell;

    #[embassy_executor::task]
    async fn try_init(
        signal: &'static Signal<CriticalSectionRawMutex, Option<InitializationError>>,
        timer: TIMG0<'static>,
        sw_int0: SoftwareInterrupt<'static, 0>,
    ) {
        let timg0 = TimerGroup::new(timer);
        esp_rtos::start(timg0.timer0, sw_int0);

        match esp_radio::init() {
            Ok(_) => signal.signal(None),
            Err(err) => signal.signal(Some(err)),
        }
    }

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    fn test_init_fails_without_scheduler(_peripherals: Peripherals) {
        // esp-rtos must be initialized before esp-radio.
        let init = esp_radio::init();

        assert!(matches!(
            init,
            Err(InitializationError::SchedulerNotInitialized),
        ));
    }

    #[test]
    fn test_init_fails_cs(p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let init = critical_section::with(|_| esp_radio::init());

        assert!(matches!(init, Err(InitializationError::InterruptsDisabled)));
    }

    #[test]
    fn test_init_fails_interrupt_free(p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let init = interrupt_free(|| esp_radio::init());

        assert!(matches!(init, Err(InitializationError::InterruptsDisabled)));
    }

    #[test]
    async fn test_init_fails_in_interrupt_executor_task(p: Peripherals) {
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);

        static EXECUTOR_CORE_0: StaticCell<InterruptExecutor<1>> = StaticCell::new();
        let executor_core0 = InterruptExecutor::new(sw_ints.software_interrupt1);
        let executor_core0 = EXECUTOR_CORE_0.init(executor_core0);

        let spawner = executor_core0.start(Priority::Priority1);

        let signal =
            mk_static!(Signal<CriticalSectionRawMutex, Option<InitializationError>>, Signal::new());

        spawner.must_spawn(try_init(signal, p.TIMG0, sw_ints.software_interrupt0));

        let res = signal.wait().await;

        assert!(matches!(
            res,
            Some(esp_radio::InitializationError::InterruptsDisabled),
        ));
    }

    #[test]
    #[cfg(soc_has_wifi)]
    fn test_wifi_can_be_initialized(mut p: Peripherals) {
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let esp_radio_ctrl =
            &*mk_static!(esp_radio::Controller<'static>, esp_radio::init().unwrap());

        // Initialize, then de-initialize wifi
        let wifi =
            esp_radio::wifi::new(&esp_radio_ctrl, p.WIFI.reborrow(), Default::default()).unwrap();
        drop(wifi);

        // Now, can we do it again?
        let _wifi =
            esp_radio::wifi::new(&esp_radio_ctrl, p.WIFI.reborrow(), Default::default()).unwrap();
    }
}
