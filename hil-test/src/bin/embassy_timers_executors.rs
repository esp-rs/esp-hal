//! Embassy timer and executor Test
//! Test that the interrupt executor correctly gives back control to thread mode
//! code.
//!
//! Reproduction and regression test for a sneaky issue.
//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod timers_executors {
    use embassy_futures::select::select;
    use embassy_time::{Duration, Ticker, Timer};
    #[cfg(not(feature = "esp32"))]
    use esp_hal::timer::systimer::SystemTimer;
    use esp_hal::{
        interrupt::{Priority, software::SoftwareInterruptControl},
        peripherals::Peripherals,
        time,
        timer::{AnyTimer, OneShotTimer, PeriodicTimer, timg::TimerGroup},
    };
    use esp_rtos::embassy::InterruptExecutor;
    use hil_test::mk_static;

    pub async fn run_test_one_shot_async() {
        let t1 = esp_hal::time::Instant::now();
        Timer::after_millis(50).await;
        Timer::after_millis(30).await;
        let t2 = esp_hal::time::Instant::now();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).as_millis() >= 80u64,
            "diff: {:?}",
            (t2 - t1).as_millis()
        );
    }

    pub fn run_test_periodic_timer<'d, T: esp_hal::timer::Timer + Into<AnyTimer<'d>>>(timer: T) {
        let mut periodic = PeriodicTimer::new(timer);

        let t1 = time::Instant::now();
        periodic.start(time::Duration::from_millis(100)).unwrap();

        periodic.wait();

        let t2 = time::Instant::now();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).as_millis() >= 100u64,
            "diff: {:?}",
            (t2 - t1).as_millis()
        );
    }

    pub fn run_test_oneshot_timer<'d, T: esp_hal::timer::Timer + Into<AnyTimer<'d>>>(timer: T) {
        let mut timer = OneShotTimer::new(timer);

        let t1 = esp_hal::time::Instant::now();
        timer.delay_millis(50);
        let t2 = esp_hal::time::Instant::now();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).as_millis() >= 50u64,
            "diff: {:?}",
            (t2 - t1).as_millis()
        );
    }

    pub async fn run_join_test() {
        let t1 = esp_hal::time::Instant::now();
        embassy_futures::join::join(Timer::after_millis(50), Timer::after_millis(30)).await;
        Timer::after_millis(50).await;
        let t2 = esp_hal::time::Instant::now();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).as_millis() >= 100u64,
            "diff: {:?}",
            (t2 - t1).as_millis()
        );
    }

    fn set_up_embassy_with_timg0(peripherals: Peripherals) {
        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);
    }

    #[cfg(not(feature = "esp32"))]
    fn set_up_embassy_with_systimer(peripherals: Peripherals) {
        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let systimer = SystemTimer::new(peripherals.SYSTIMER);
        esp_rtos::start(systimer.alarm0, sw_int.software_interrupt0);
    }

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    async fn test_one_shot_timg(peripherals: Peripherals) {
        set_up_embassy_with_timg0(peripherals);

        run_test_one_shot_async().await;
    }

    #[test]
    #[cfg(not(feature = "esp32"))]
    async fn test_one_shot_systimer(peripherals: Peripherals) {
        set_up_embassy_with_systimer(peripherals);

        run_test_one_shot_async().await;
    }

    #[test]
    fn test_periodic_timg(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);

        run_test_periodic_timer(timg0.timer0);
    }

    #[test]
    #[cfg(not(feature = "esp32"))]
    fn test_periodic_systimer(peripherals: Peripherals) {
        let systimer = SystemTimer::new(peripherals.SYSTIMER);

        run_test_periodic_timer(systimer.alarm0);
    }

    #[test]
    fn test_periodic_oneshot_timg(peripherals: Peripherals) {
        let mut timg0 = TimerGroup::new(peripherals.TIMG0);
        run_test_periodic_timer(timg0.timer0.reborrow());
        run_test_oneshot_timer(timg0.timer0.reborrow());
    }

    #[test]
    #[cfg(not(feature = "esp32"))]
    fn test_periodic_oneshot_systimer(peripherals: Peripherals) {
        let mut systimer = SystemTimer::new(peripherals.SYSTIMER);
        run_test_periodic_timer(systimer.alarm0.reborrow());
        run_test_oneshot_timer(systimer.alarm0.reborrow());
    }

    #[test]
    async fn test_join_timg(peripherals: Peripherals) {
        set_up_embassy_with_timg0(peripherals);

        run_join_test().await;
    }

    #[test]
    #[cfg(not(feature = "esp32"))]
    async fn test_join_systimer(peripherals: Peripherals) {
        set_up_embassy_with_systimer(peripherals);

        run_join_test().await;
    }

    /// Test that the ticker works in tasks ran by the interrupt executors.
    #[test]
    async fn test_interrupt_executor(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        let executor = mk_static!(
            InterruptExecutor<2>,
            InterruptExecutor::new(sw_int.software_interrupt2)
        );

        #[embassy_executor::task]
        async fn e_task30ms() {
            Timer::after_millis(30).await;
        }

        #[embassy_executor::task]
        async fn test_interrupt_executor_invoker() {
            let outcome = async {
                let mut ticker = Ticker::every(Duration::from_millis(30));

                let t1 = esp_hal::time::Instant::now();
                ticker.next().await;
                ticker.next().await;
                ticker.next().await;
                let t2 = esp_hal::time::Instant::now();

                assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
                assert!(
                    (t2 - t1).as_micros() >= 85000u64,
                    "diff: {:?}",
                    (t2 - t1).as_micros()
                );
            };

            embedded_test::export::check_outcome(outcome.await);
        }

        let spawner_int = executor.start(Priority::Priority3);
        spawner_int.must_spawn(test_interrupt_executor_invoker());

        let spawner = embassy_executor::SendSpawner::for_current_executor().await;
        spawner.must_spawn(e_task30ms());

        // The test ends once the interrupt executor's task has finished
        loop {}
    }

    /// Test that timg0 and systimer don't have vastly different tick rates.
    #[test]
    async fn tick_test_timer_tick_rates(peripherals: Peripherals) {
        set_up_embassy_with_timg0(peripherals);

        // We are retrying 5 times because probe-rs polling RTT may introduce some
        // jitter.
        for _ in 0..5 {
            let t1 = esp_hal::time::Instant::now();

            let mut ticker = Ticker::every(Duration::from_hz(100_000));
            for _ in 0..2000 {
                ticker.next().await;
            }
            let t2 = esp_hal::time::Instant::now();

            assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
            let duration = (t2 - t1).as_micros();

            assert!(duration >= 19000, "diff: {:?}", (t2 - t1).as_micros());
            if duration <= 21000 {
                return;
            }
        }

        assert!(false, "Test failed after 5 retries");
    }

    /// Test that timg0 and systimer don't have vastly different tick rates.
    #[test]
    async fn test_that_a_very_long_wakeup_does_not_panic(peripherals: Peripherals) {
        set_up_embassy_with_timg0(peripherals);

        select(
            Timer::after(Duration::from_micros(u64::MAX / 2)),
            embassy_futures::yield_now(), // we don't actually want to wait forever
        )
        .await;
    }
}

#[embedded_test::tests(default_timeout = 3)]
mod interrupt_executor {
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
    use esp_hal::{
        interrupt::{
            Priority,
            software::{SoftwareInterrupt, SoftwareInterruptControl},
        },
        timer::timg::TimerGroup,
    };
    #[cfg(multi_core)]
    use esp_hal::{
        peripherals::CPU_CTRL,
        system::{Cpu, CpuControl, Stack},
    };
    use esp_rtos::embassy::{Callbacks, Executor, InterruptExecutor};
    use hil_test::mk_static;

    #[embassy_executor::task]
    async fn responder_task(
        signal: &'static Signal<CriticalSectionRawMutex, ()>,
        response: &'static Signal<CriticalSectionRawMutex, ()>,
    ) {
        response.signal(());
        loop {
            signal.wait().await;
            response.signal(());
        }
    }

    #[embassy_executor::task]
    async fn tester_task(
        signal: &'static Signal<CriticalSectionRawMutex, ()>,
        response: &'static Signal<CriticalSectionRawMutex, ()>,
    ) {
        response.wait().await;
        for _ in 0..3 {
            signal.signal(());
            response.wait().await;
        }
        embedded_test::export::check_outcome(());
    }

    #[embassy_executor::task]
    #[cfg(multi_core)]
    async fn tester_task_multi_core(
        signal: &'static Signal<CriticalSectionRawMutex, ()>,
        response: &'static Signal<CriticalSectionRawMutex, ()>,
    ) {
        response.wait().await;
        for _ in 0..3 {
            signal.signal(());
            response.wait().await;
        }

        unsafe {
            // Park the second core, we don't need it anymore
            CpuControl::new(CPU_CTRL::steal()).park_core(Cpu::AppCpu);
        }
        embedded_test::export::check_outcome(());
    }

    struct Context {
        #[cfg(multi_core)]
        sw_int1: SoftwareInterrupt<'static, 1>,
        sw_int2: SoftwareInterrupt<'static, 2>,
        #[cfg(multi_core)]
        cpu_control: CPU_CTRL<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        Context {
            #[cfg(multi_core)]
            sw_int1: sw_int.software_interrupt1,
            sw_int2: sw_int.software_interrupt2,
            #[cfg(multi_core)]
            cpu_control: peripherals.CPU_CTRL,
        }
    }

    #[test]
    fn run_test_with_callbacks_api(ctx: Context) {
        let interrupt_executor =
            mk_static!(InterruptExecutor<2>, InterruptExecutor::new(ctx.sw_int2));
        let signal = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let response = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let spawner = interrupt_executor.start(Priority::Priority3);
        spawner.must_spawn(responder_task(signal, response));

        let thread_executor = mk_static!(Executor, Executor::new());

        struct NoCallbacks;

        impl Callbacks for NoCallbacks {
            fn before_poll(&mut self) {}
            fn on_idle(&mut self) {}
        }

        let callbacks = NoCallbacks;

        thread_executor.run_with_callbacks(
            |spawner| {
                spawner.must_spawn(tester_task(signal, response));
            },
            callbacks,
        )
    }

    #[test]
    fn run_interrupt_executor_test(ctx: Context) {
        let interrupt_executor =
            mk_static!(InterruptExecutor<2>, InterruptExecutor::new(ctx.sw_int2));
        let signal = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let response = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let spawner = interrupt_executor.start(Priority::Priority3);
        spawner.must_spawn(responder_task(signal, response));

        let thread_executor = mk_static!(Executor, Executor::new());
        thread_executor.run(|spawner| {
            spawner.must_spawn(tester_task(signal, response));
        })
    }

    #[test]
    #[cfg(multi_core)]
    fn run_interrupt_executor_test_on_core_1(ctx: Context) {
        let app_core_stack = mk_static!(Stack<8192>, Stack::new());
        let response = &*mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let signal = &*mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        esp_rtos::start_second_core(ctx.cpu_control, ctx.sw_int1, app_core_stack, || {
            let interrupt_executor =
                mk_static!(InterruptExecutor<2>, InterruptExecutor::new(ctx.sw_int2));

            let spawner = interrupt_executor.start(Priority::Priority3);

            spawner.spawn(responder_task(signal, response)).unwrap();
        });

        let thread_executor = mk_static!(Executor, Executor::new());
        thread_executor.run(|spawner| {
            spawner.must_spawn(tester_task_multi_core(signal, response));
        })
    }

    #[test]
    #[cfg(multi_core)]
    fn run_thread_executor_test_on_core_1(ctx: Context) {
        let app_core_stack = mk_static!(Stack<8192>, Stack::new());
        let signal = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
        let response = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        esp_rtos::start_second_core(ctx.cpu_control, ctx.sw_int1, app_core_stack, || {
            let executor = mk_static!(Executor, Executor::new());
            executor.run(|spawner| {
                spawner.spawn(responder_task(signal, response)).ok();
            });
        });

        let thread_executor = mk_static!(Executor, Executor::new());
        thread_executor.run(|spawner| {
            spawner.must_spawn(tester_task_multi_core(signal, response));
        })
    }
}

#[cfg(any(esp32, esp32c3, esp32c6, esp32h2, esp32s2, esp32s3))]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod interrupt_spi_dma {
    use embassy_time::{Duration, Instant, Timer};
    use esp_hal::{
        Blocking,
        dma::{DmaRxBuf, DmaTxBuf},
        dma_buffers,
        interrupt::{Priority, software::SoftwareInterruptControl},
        spi::{
            Mode,
            master::{Config, Spi},
        },
        time::Rate,
        timer::timg::TimerGroup,
    };
    use esp_rtos::embassy::InterruptExecutor;
    use hil_test::mk_static;
    use portable_atomic::AtomicBool;

    static STOP_INTERRUPT_TASK: AtomicBool = AtomicBool::new(false);
    static INTERRUPT_TASK_WORKING: AtomicBool = AtomicBool::new(false);

    #[cfg(any(esp32, esp32s2, esp32s3))]
    #[embassy_executor::task]
    async fn interrupt_driven_task(spi: esp_hal::spi::master::SpiDma<'static, Blocking>) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(128);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = spi.with_buffers(dma_rx_buf, dma_tx_buf).into_async();

        loop {
            let mut buffer: [u8; 8] = [0; 8];

            INTERRUPT_TASK_WORKING.store(true, portable_atomic::Ordering::Relaxed);
            spi.transfer_in_place_async(&mut buffer).await.unwrap();
            INTERRUPT_TASK_WORKING.store(false, portable_atomic::Ordering::Relaxed);

            if STOP_INTERRUPT_TASK.load(portable_atomic::Ordering::Relaxed) {
                break;
            }

            Timer::after(Duration::from_millis(1)).await;
        }
    }

    #[cfg(not(any(esp32, esp32s2, esp32s3)))]
    #[embassy_executor::task]
    async fn interrupt_driven_task(i2s_tx: esp_hal::i2s::master::I2s<'static, Blocking>) {
        let (_, _, _, tx_descriptors) = dma_buffers!(128);

        let mut i2s_tx = i2s_tx.into_async().i2s_tx.build(tx_descriptors);

        loop {
            let mut buffer: [u8; 8] = [0; 8];

            INTERRUPT_TASK_WORKING.store(true, portable_atomic::Ordering::Relaxed);
            i2s_tx.write_dma_async(&mut buffer).await.unwrap();
            INTERRUPT_TASK_WORKING.store(false, portable_atomic::Ordering::Relaxed);

            if STOP_INTERRUPT_TASK.load(portable_atomic::Ordering::Relaxed) {
                break;
            }

            Timer::after(Duration::from_millis(1)).await;
        }
    }

    #[test]
    async fn dma_does_not_lock_up_when_used_in_different_executors() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel1 = peripherals.DMA_SPI2;
                let dma_channel2 = peripherals.DMA_SPI3;
            } else {
                let dma_channel1 = peripherals.DMA_CH0;
                let dma_channel2 = peripherals.DMA_CH1;
            }
        }

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(1024);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let (_, mosi) = hil_test::common_test_pins!(peripherals);

        let mut spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(Rate::from_khz(10000))
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_miso(unsafe { mosi.clone_unchecked() })
        .with_mosi(mosi)
        .with_dma(dma_channel1)
        .with_buffers(dma_rx_buf, dma_tx_buf)
        .into_async();

        #[cfg(any(esp32, esp32s2, esp32s3))]
        let other_peripheral = Spi::new(
            peripherals.SPI3,
            Config::default()
                .with_frequency(Rate::from_khz(10000))
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_dma(dma_channel2);

        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        let other_peripheral = esp_hal::i2s::master::I2s::new(
            peripherals.I2S0,
            dma_channel2,
            esp_hal::i2s::master::Config::new_tdm_philips()
                .with_sample_rate(Rate::from_khz(8))
                .with_data_format(esp_hal::i2s::master::DataFormat::Data16Channel16)
                .with_channels(esp_hal::i2s::master::Channels::STEREO),
        )
        .unwrap();

        let interrupt_executor = mk_static!(
            InterruptExecutor<1>,
            InterruptExecutor::new(sw_int.software_interrupt1)
        );

        let spawner = interrupt_executor.start(Priority::Priority3);

        spawner
            .spawn(interrupt_driven_task(other_peripheral))
            .unwrap();

        let start = Instant::now();
        let mut buffer: [u8; 1024] = [0; 1024];
        let mut dst_buffer: [u8; 1024] = [0; 1024];
        let mut i = 0;
        loop {
            buffer.fill(i);
            dst_buffer.fill(i.wrapping_add(1));
            spi.transfer_async(&mut dst_buffer, &buffer).await.unwrap();
            // make sure the transfer didn't end prematurely
            assert!(dst_buffer.iter().all(|&v| v == i));

            if start.elapsed() > Duration::from_secs(1) {
                break;
            }

            i = i.wrapping_add(1);
        }

        // make sure the other peripheral didn't get stuck
        STOP_INTERRUPT_TASK.store(true, portable_atomic::Ordering::Relaxed);
        while INTERRUPT_TASK_WORKING.load(portable_atomic::Ordering::Relaxed) {}
    }

    // Reproducer of https://github.com/esp-rs/esp-hal/issues/2369
    #[cfg(multi_core)]
    #[test]
    async fn dma_does_not_lock_up_on_core_1() {
        use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
        use esp_hal::{
            peripherals::{CPU_CTRL, SPI2},
            system::{Cpu, CpuControl, Stack},
        };

        cfg_if::cfg_if! {
            if #[cfg(dma_kind = "pdma")] {
                type DmaChannel<'a> = esp_hal::peripherals::DMA_SPI2<'a>;
            } else {
                type DmaChannel<'a> = esp_hal::peripherals::DMA_CH0<'a>;
            }
        }

        const BUFFER_SIZE: usize = 256;

        pub struct SpiPeripherals {
            pub spi: SPI2<'static>,
            pub dma_channel: DmaChannel<'static>,
        }

        #[embassy_executor::task]
        async fn run_spi(
            peripherals: SpiPeripherals,
            finished: &'static Signal<CriticalSectionRawMutex, ()>,
        ) {
            let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(3200);
            let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
            let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

            let mut spi = Spi::new(
                peripherals.spi,
                Config::default()
                    .with_frequency(Rate::from_khz(100))
                    .with_mode(Mode::_0),
            )
            .unwrap()
            .with_dma(peripherals.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf)
            .into_async();

            let send_buffer = mk_static!([u8; BUFFER_SIZE], [0u8; BUFFER_SIZE]);
            loop {
                let mut buffer = [0; 8];
                embedded_hal_async::spi::SpiBus::transfer(&mut spi, &mut buffer, send_buffer)
                    .await
                    .unwrap();
                finished.signal(());
            }
        }

        let peripherals = esp_hal::init(esp_hal::Config::default());

        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        cfg_if::cfg_if! {
            if #[cfg(dma_kind = "pdma")] {
                let dma_channel = peripherals.DMA_SPI2;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let transfer_finished = &*mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let spi_peripherals = SpiPeripherals {
            spi: peripherals.SPI2,
            dma_channel,
        };

        let app_core_stack = mk_static!(Stack<8192>, Stack::new());

        esp_rtos::start_second_core(
            peripherals.CPU_CTRL,
            sw_int.software_interrupt1,
            app_core_stack,
            || {
                use esp_hal::interrupt::Priority;
                let software_interrupt = sw_int.software_interrupt2;
                let hp_executor = mk_static!(
                    InterruptExecutor<2>,
                    InterruptExecutor::new(software_interrupt)
                );
                let high_pri_spawner = hp_executor.start(Priority::Priority2);

                // spi runs as high priority task
                high_pri_spawner
                    .spawn(run_spi(spi_peripherals, transfer_finished))
                    .ok();
            },
        );

        // Wait for a few SPI transfers to happen
        for _ in 0..5 {
            transfer_finished.wait().await;
        }

        // make sure the other peripheral didn't get stuck
        STOP_INTERRUPT_TASK.store(true, portable_atomic::Ordering::Relaxed);
        while INTERRUPT_TASK_WORKING.load(portable_atomic::Ordering::Relaxed) {}

        unsafe {
            // Park the second core, we don't need it anymore
            CpuControl::new(CPU_CTRL::steal()).park_core(Cpu::AppCpu);
        }
    }
}
