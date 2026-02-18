#[embedded_test::tests(default_timeout = 3, executor = esp_rtos::embassy::Executor::new())]
mod init_tests {

    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
    #[cfg(soc_has_wifi)]
    use esp_hal::peripherals::WIFI;
    #[cfg(riscv)]
    use esp_hal::riscv::interrupt::free as interrupt_free;
    #[cfg(xtensa)]
    use esp_hal::xtensa_lx::interrupt::free as interrupt_free;
    use esp_hal::{
        clock::CpuClock,
        interrupt::{Priority, software::SoftwareInterruptControl},
        peripherals::Peripherals,
        timer::timg::TimerGroup,
    };
    #[cfg(bt_driver_supported)]
    use esp_radio::ble::controller::BleConnector;
    #[cfg(soc_has_wifi)]
    use esp_radio::wifi::WifiError;
    use esp_rtos::embassy::InterruptExecutor;
    use hil_test::mk_static;
    use static_cell::StaticCell;

    #[embassy_executor::task]
    #[cfg(soc_has_wifi)]
    async fn try_init(
        signal: &'static Signal<CriticalSectionRawMutex, Option<WifiError>>,
        wifi_peripheral: WIFI<'static>,
    ) {
        match esp_radio::wifi::new(wifi_peripheral, Default::default()) {
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

    // Test we get an error when attempting to initialize esp-radio with interrupts
    // disabled in common ways

    #[test]
    #[should_panic]
    #[cfg(soc_has_wifi)]
    fn test_init_fails_without_scheduler(p: Peripherals) {
        // esp-rtos must be initialized before esp-radio.
        let _ = esp_radio::wifi::new(p.WIFI, Default::default());
    }

    #[test]
    #[cfg(soc_has_wifi)]
    fn test_init_fails_cs(p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let init = critical_section::with(|_| esp_radio::wifi::new(p.WIFI, Default::default()));

        match init {
            Ok(_) => defmt::info!("Initialized wifi in critical section"),
            Err(ref e) => defmt::info!("Failed to initialize wifi in critical section: {:?}", e),
        }

        assert!(matches!(init, Err(WifiError::Unsupported)));
    }

    #[test]
    #[cfg(soc_has_wifi)]
    fn test_init_fails_interrupt_free(p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let init = interrupt_free(|| esp_radio::wifi::new(p.WIFI, Default::default()));

        assert!(matches!(init, Err(WifiError::Unsupported)));
    }

    #[test]
    #[cfg(soc_has_wifi)]
    async fn test_init_fails_in_interrupt_executor_task(p: Peripherals) {
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);

        static EXECUTOR_CORE_0: StaticCell<InterruptExecutor<1>> = StaticCell::new();
        let executor_core0 = InterruptExecutor::new(sw_ints.software_interrupt1);
        let executor_core0 = EXECUTOR_CORE_0.init(executor_core0);

        let spawner = executor_core0.start(Priority::Priority1);

        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let signal = mk_static!(Signal<CriticalSectionRawMutex, Option<WifiError>>, Signal::new());

        spawner.must_spawn(try_init(signal, p.WIFI));

        let res = signal.wait().await;

        assert!(matches!(res, Some(WifiError::Unsupported)));
    }

    #[test]
    #[cfg(soc_has_wifi)]
    fn test_wifi_can_be_initialized(mut p: Peripherals) {
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        // Initialize, then de-initialize wifi
        let wifi = esp_radio::wifi::new(p.WIFI.reborrow(), Default::default()).unwrap();
        drop(wifi);

        // Now, can we do it again?
        let _wifi = esp_radio::wifi::new(p.WIFI.reborrow(), Default::default()).unwrap();
    }

    #[test]
    #[cfg(soc_has_wifi)]
    #[cfg(bt_driver_supported)]
    fn test_init_and_drop(mut p: Peripherals) {
        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        // Initialize BLE and WiFi then drop BLE
        let connector = BleConnector::new(p.BT.reborrow(), Default::default()).unwrap();
        let wifi = esp_radio::wifi::new(p.WIFI.reborrow(), Default::default()).unwrap();
        drop(connector);

        // Re-initialize BLE and drop WiFi and BLE
        let connector = BleConnector::new(p.BT.reborrow(), Default::default()).unwrap();
        drop(wifi);
        drop(connector);
    }

    #[test]
    #[cfg(soc_has_wifi)]
    #[cfg(bt_driver_supported)]
    fn test_create_ble_wifi_drop_ble_wifi_create_wifi_ble(mut p: Peripherals) {
        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        // Initialize WiFi and BLE then drop BLE and WiFi
        let wifi = esp_radio::wifi::new(p.WIFI.reborrow(), Default::default()).unwrap();
        let connector = BleConnector::new(p.BT.reborrow(), Default::default()).unwrap();

        drop(connector);
        drop(wifi);

        // Re-initialize WiFi and BLE then drop WiFi
        let wifi = esp_radio::wifi::new(p.WIFI.reborrow(), Default::default()).unwrap();
        let _connector = BleConnector::new(p.BT.reborrow(), Default::default()).unwrap();

        drop(wifi);
    }
}
