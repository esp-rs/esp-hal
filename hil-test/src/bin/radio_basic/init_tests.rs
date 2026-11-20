#[embedded_test::tests(default_timeout = 3, executor = esp_rtos::embassy::Executor::new())]
mod init_tests {

    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
    use embassy_time::{Duration, Instant, Timer};
    #[cfg(soc_has_wifi)]
    use esp_hal::peripherals::WIFI;
    #[cfg(riscv)]
    use esp_hal::riscv::interrupt::free as interrupt_free;
    #[cfg(xtensa)]
    use esp_hal::xtensa_lx::interrupt::free as interrupt_free;
    use esp_hal::{
        clock::CpuClock,
        interrupt::Priority,
        peripherals::Peripherals,
        timer::timg::TimerGroup,
    };
    #[cfg(all(bt_driver_supported, feature = "esp-radio-unstable"))]
    use esp_radio::ble::controller::BleConnector;
    #[cfg(soc_has_wifi)]
    use esp_radio::wifi::WifiError;
    use esp_rtos::embassy::InterruptExecutor;
    use hil_test::mk_static;
    use portable_atomic::{AtomicU32, Ordering};
    use static_cell::StaticCell;

    #[embassy_executor::task]
    #[cfg(soc_has_wifi)]
    async fn try_init(
        signal: &'static Signal<CriticalSectionRawMutex, Option<WifiError>>,
        wifi_peripheral: WIFI<'static>,
    ) {
        match esp_radio::wifi::WifiController::new(wifi_peripheral, Default::default()) {
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
    #[timeout(6)]
    #[cfg(soc_has_wifi)]
    fn test_init_fails_without_scheduler(p: Peripherals) {
        // esp-rtos must be initialized before esp-radio.
        let _ = esp_radio::wifi::WifiController::new(p.WIFI, Default::default());
    }

    #[test]
    #[should_panic]
    #[timeout(6)]
    #[cfg(soc_has_wifi)]
    fn test_init_fails_cs(p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let _ = critical_section::with(|_| {
            esp_radio::wifi::WifiController::new(p.WIFI, Default::default())
        });
    }

    #[test]
    #[should_panic]
    #[timeout(6)]
    #[cfg(soc_has_wifi)]
    fn test_init_fails_interrupt_free(p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let _ = interrupt_free(|| esp_radio::wifi::WifiController::new(p.WIFI, Default::default()));
    }

    #[test]
    #[should_panic]
    #[timeout(6)]
    #[cfg(soc_has_wifi)]
    async fn test_init_fails_in_interrupt_executor_task(p: Peripherals) {
        static EXECUTOR_CORE_0: StaticCell<InterruptExecutor<2>> = StaticCell::new();
        let executor_core0 = InterruptExecutor::new(p.FROM_CPU_INTR2);
        let executor_core0 = EXECUTOR_CORE_0.init(executor_core0);

        let spawner = executor_core0.start(Priority::Priority1);

        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let signal = mk_static!(Signal<CriticalSectionRawMutex, Option<WifiError>>, Signal::new());

        spawner.spawn(try_init(signal, p.WIFI).unwrap());

        let _ = signal.wait().await;
    }

    #[test]
    #[cfg(soc_has_wifi)]
    fn test_wifi_can_be_initialized(mut p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        // Initialize, then de-initialize wifi
        let wifi =
            esp_radio::wifi::WifiController::new(p.WIFI.reborrow(), Default::default()).unwrap();
        drop(wifi);

        // Now, can we do it again?
        let _wifi =
            esp_radio::wifi::WifiController::new(p.WIFI.reborrow(), Default::default()).unwrap();
    }

    #[test]
    #[cfg(soc_has_wifi)]
    #[cfg(all(bt_driver_supported, feature = "esp-radio-unstable"))]
    fn test_init_and_drop(mut p: Peripherals) {
        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        // Initialize BLE and WiFi then drop BLE
        let connector = BleConnector::new(p.BT.reborrow(), Default::default()).unwrap();
        let wifi =
            esp_radio::wifi::WifiController::new(p.WIFI.reborrow(), Default::default()).unwrap();
        drop(connector);

        // Re-initialize BLE and drop WiFi and BLE
        let connector = BleConnector::new(p.BT.reborrow(), Default::default()).unwrap();
        drop(wifi);
        drop(connector);
    }

    #[test]
    #[cfg(soc_has_wifi)]
    #[cfg(all(bt_driver_supported, feature = "esp-radio-unstable"))]
    fn test_create_ble_wifi_drop_ble_wifi_create_wifi_ble(mut p: Peripherals) {
        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        // Initialize WiFi and BLE then drop BLE and WiFi
        let wifi =
            esp_radio::wifi::WifiController::new(p.WIFI.reborrow(), Default::default()).unwrap();
        let connector = BleConnector::new(p.BT.reborrow(), Default::default()).unwrap();

        drop(connector);
        drop(wifi);

        // Re-initialize WiFi and BLE then drop WiFi
        let wifi =
            esp_radio::wifi::WifiController::new(p.WIFI.reborrow(), Default::default()).unwrap();
        let _connector = BleConnector::new(p.BT.reborrow(), Default::default()).unwrap();

        drop(wifi);
    }

    #[test]
    #[cfg(soc_has_wifi)]
    fn interrupts_run_while_the_phy_calibrates(p: Peripherals) {
        static LONGEST_GAP_US: AtomicU32 = AtomicU32::new(0);

        #[embassy_executor::task]
        async fn measure_gaps() {
            let mut last = Instant::now();
            loop {
                Timer::after(Duration::from_millis(1)).await;
                let now = Instant::now();
                LONGEST_GAP_US.fetch_max((now - last).as_micros() as u32, Ordering::Relaxed);
                last = now;
            }
        }

        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        static EXECUTOR: StaticCell<InterruptExecutor<2>> = StaticCell::new();
        let spawner = EXECUTOR
            .init(InterruptExecutor::new(p.FROM_CPU_INTR2))
            .start(Priority::Priority3);
        spawner.spawn(measure_gaps().unwrap());

        let _controller = esp_radio::wifi::WifiController::new(p.WIFI, Default::default()).unwrap();

        let longest_gap_us = LONGEST_GAP_US.load(Ordering::Relaxed);
        hil_test::assert!(
            longest_gap_us < 20_000,
            "Interrupts were blocked for {} us",
            longest_gap_us
        );
    }

    // Wi-Fi starts in a higher-priority thread while BLE calibrates. It must wait, and its clocks
    // must survive the calibration's clock restore.
    #[test]
    #[timeout(6)]
    #[cfg(all(
        any(esp32c5, esp32c6, esp32c61, esp32s31),
        feature = "esp-radio-unstable"
    ))]
    fn wifi_waits_for_ble_calibration(p: Peripherals) {
        use esp_hal::peripherals::MODEM_SYSCON;
        use esp_rtos::{CurrentThreadHandle, thread::ThreadSpawner};

        // `clk_wifibb_*_en`, which the calibration enables. Wi-Fi does not enable
        // `clk_wifibb_80x1_en` (bit 7) on the ESP32-S31.
        const WIFI_BASEBAND_CLOCKS: u32 = 0x1ff & !(1 << 7);
        fn wifi_baseband_clocks() -> u32 {
            MODEM_SYSCON::regs().clk_conf1().read().bits() & WIFI_BASEBAND_CLOCKS
        }

        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        // Reset and earlier tests leave them on.
        MODEM_SYSCON::regs()
            .clk_conf1()
            .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BASEBAND_CLOCKS) });

        let wifi = p.WIFI;
        let wifi_thread = ThreadSpawner::new(16384).with_priority(10).spawn(move || {
            // Waits up to 2 s for the calibration, which enables the Wi-Fi baseband clocks.
            let started_during_calibration = (0..4000).any(|_| {
                CurrentThreadHandle::get().delay(esp_hal::time::Duration::from_micros(500));
                wifi_baseband_clocks() == WIFI_BASEBAND_CLOCKS
            });

            let controller =
                esp_radio::wifi::WifiController::new(wifi, Default::default()).unwrap();
            (controller, started_during_calibration)
        });

        let _connector = BleConnector::new(p.BT, Default::default()).unwrap();
        let ((_controller, started_during_calibration), _) = wifi_thread.join();

        hil_test::assert!(
            started_during_calibration,
            "The calibration did not let the Wi-Fi thread run"
        );
        hil_test::assert_eq!(wifi_baseband_clocks(), WIFI_BASEBAND_CLOCKS);
    }
}
