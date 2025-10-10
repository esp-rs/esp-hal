//! Miscellaneous simple tests
//!
//! Clock Monitor Test
//!
//! Ensure invariants of locks are upheld.
//!
//! Async Delay Test
//!     Specifically tests the various implementations of the
//!     `embedded_hal_async::delay::DelayNs` trait.
//!     This test does not configure embassy, as it doesn't need a timer queue
//!     implementation or an embassy time driver.
//!
//! DMA macro tests
//!
//! DMA Mem2Mem Tests
//!
//! Initialization tests
//!
//! The goal of this test suite is to collect smaller, simpler test cases, to keep the overall
//! number of test suites low(er).

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

#[embedded_test::tests(default_timeout = 2)]
mod simple {
    use embedded_hal::delay::DelayNs;
    use esp_hal::{
        delay::Delay,
        peripherals::Peripherals,
        rom::{crc, md5},
        time::{Duration, Instant},
    };
    use hil_test as _;

    fn time_moves_forward_during<F: FnOnce(Context)>(ctx: Context, f: F) {
        let t1 = Instant::now();
        f(ctx);
        let t2 = Instant::now();

        assert!(t2 > t1);
    }

    struct Context {
        p: Peripherals,
    }

    #[init]
    fn init() -> Context {
        Context {
            p: esp_hal::init(esp_hal::Config::default()),
        }
    }

    // Test time

    #[test]
    fn duration_since_epoch_is_not_relative_to_now() {
        let now = Instant::EPOCH;

        Delay::new().delay_ns(10_000);

        assert_eq!(now.duration_since_epoch(), Duration::ZERO);
    }

    #[test]
    fn large_instant_difference_does_not_panic() {
        assert_eq!(
            (Instant::EPOCH + Duration::MAX).duration_since_epoch(),
            Duration::MAX
        );
    }

    #[cfg(systimer)]
    #[test]
    fn test_current_time_construct_systimer(ctx: Context) {
        time_moves_forward_during(ctx, |ctx| {
            // construct the timer in between calls to current_time
            let _ = esp_hal::timer::systimer::SystemTimer::new(ctx.p.SYSTIMER);
        })
    }

    #[cfg(esp32)]
    #[test]
    fn test_current_time_construct_timg0(ctx: Context) {
        time_moves_forward_during(ctx, |ctx| {
            // construct the timer in between calls to current_time
            let _ = esp_hal::timer::timg::TimerGroup::new(ctx.p.TIMG0);
        })
    }

    #[test]
    fn delay_ns() {
        let t1 = Instant::now();
        Delay::new().delay_ns(600_000);
        let t2 = Instant::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_micros() >= 600u64,
            "diff: {:?}",
            (t2 - t1).as_micros()
        );
    }

    #[test]
    fn delay_70millis() {
        let t1 = Instant::now();
        Delay::new().delay_millis(70);
        let t2 = Instant::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_millis() >= 70u64,
            "diff: {:?}",
            (t2 - t1).as_millis()
        );
    }

    #[test]
    fn delay_1_500us() {
        let t1 = Instant::now();
        Delay::new().delay_us(1_500);
        let t2 = Instant::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_micros() >= 1_500u64,
            "diff: {:?}",
            (t2 - t1).as_micros()
        );
    }

    #[test]
    fn delay_300ms() {
        let t1 = Instant::now();
        Delay::new().delay_ms(300);
        let t2 = Instant::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_millis() >= 300u64,
            "diff: {:?}",
            (t2 - t1).as_millis()
        );
    }

    // Test ROM functions

    #[test]
    fn test_crc() {
        let data = "123456789";

        let crc_hdlc = crc::crc32_le(!0xffffffff, data.as_ref());
        let crc_bzip2 = crc::crc32_be(!0xffffffff, data.as_ref());
        let crc_mpeg2 = !crc::crc32_be(!0xffffffff, data.as_ref());
        let crc_cksum = crc::crc32_be(!0, data.as_ref());
        let crc_kermit = !crc::crc16_le(!0, data.as_ref());
        let crc_genibus = crc::crc16_be(!0xffff, data.as_ref());
        let crc_rohc = !crc::crc8_le(!0xff, data.as_ref());
        let crc_smbus = !crc::crc8_be(!0, data.as_ref());

        assert_eq!(crc_hdlc, 0xcbf43926);
        assert_eq!(crc_bzip2, 0xfc891918);
        assert_eq!(crc_mpeg2, 0x0376e6e7);
        assert_eq!(crc_cksum, 0x765e7680);
        assert_eq!(crc_kermit, 0x2189);
        assert_eq!(crc_genibus, 0xd64e);
        assert_eq!(crc_rohc, 0xd0);
        assert_eq!(crc_smbus, 0xf4);
    }

    #[test]
    fn test_crc_rom_function() {
        let crc = esp_bootloader_esp_idf::Crc32ForTesting::new();
        let res = crc.crc(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);
        assert_eq!(res, 436745307);
    }

    #[test]
    fn test_md5() {
        let sentence = "The quick brown fox jumps over a lazy dog";

        let mut md5_ctx = md5::Context::new();
        let mut it = sentence.split_whitespace().peekable();
        while let Some(word) = it.next() {
            md5_ctx.consume(word);
            if it.peek().is_some() {
                md5_ctx.consume(" ");
            }
        }
        let md5_digest = md5_ctx.compute();

        let expected_md5_digest = [
            0x30, 0xde, 0xd8, 0x07, 0xd6, 0x5e, 0xe0, 0x37, 0x0f, 0xc6, 0xd7, 0x3d, 0x6a, 0xb5,
            0x5a, 0x95,
        ];

        assert_eq!(expected_md5_digest, *md5_digest);
    }

    #[test]
    #[cfg(soc_has_usb_device)]
    fn creating_peripheral_does_not_break_debug_connection(ctx: Context) {
        use esp_hal::usb_serial_jtag::UsbSerialJtag;

        _ = UsbSerialJtag::new(ctx.p.USB_DEVICE).into_async().split();
    }
}

#[embedded_test::tests(default_timeout = 3)]
mod clock_monitor {
    use esp_hal::rtc_cntl::Rtc;

    struct Context<'a> {
        rtc: Rtc<'a>,
    }
    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let rtc = Rtc::new(peripherals.LPWR);

        Context { rtc }
    }

    #[test]
    fn test_estimated_clock(mut ctx: Context<'static>) {
        let target_frequency = if cfg!(esp32c2) {
            26
        } else if cfg!(esp32h2) {
            32
        } else {
            40
        };

        // The internal RC oscillators are not very accurate at all. Leave a 20% acceptance range
        // around the expected value.
        let twenty_percent = 20 * target_frequency / 100;
        let expected_range =
            (target_frequency - twenty_percent)..=(target_frequency + twenty_percent);

        let measured_frequency = ctx.rtc.estimate_xtal_frequency();
        hil_test::assert!(
            expected_range.contains(&measured_frequency),
            "Measured frequency: {}",
            measured_frequency
        );
    }
}

#[embedded_test::tests(default_timeout = 3)]
mod critical_section {
    use esp_hal::{
        delay::Delay,
        interrupt::{
            InterruptHandler,
            Priority,
            software::{SoftwareInterrupt, SoftwareInterruptControl},
        },
        peripherals::Peripherals,
        sync::RawPriorityLimitedMutex,
    };
    use esp_sync::NonReentrantMutex;
    use portable_atomic::{AtomicU32, Ordering};

    fn test_access_at_priority(peripherals: Peripherals, priority: Priority) {
        static LOCK: RawPriorityLimitedMutex = RawPriorityLimitedMutex::new(Priority::Priority1);

        extern "C" fn access<const INT: u8>() {
            unsafe { SoftwareInterrupt::<INT>::steal().reset() };
            LOCK.lock(|| {});
            embedded_test::export::check_outcome(());
        }

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let mut prio_2_interrupt = sw_ints.software_interrupt1;

        prio_2_interrupt.set_interrupt_handler(InterruptHandler::new(access::<1>, priority));

        prio_2_interrupt.raise();
        loop {}
    }

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    fn critical_section_is_reentrant() {
        let mut flag = false;

        critical_section::with(|_| {
            critical_section::with(|_| {
                flag = true;
            });
        });

        assert!(flag);
    }

    #[test]
    fn non_reentrant_mutex_can_provide_mutable_access() {
        let flag = NonReentrantMutex::new(false);

        flag.with(|f| {
            *f = true;
        });
        flag.with(|f| {
            assert!(*f);
        });
    }

    #[test]
    #[should_panic]
    fn non_reentrant_mutex_is_not_reentrant() {
        let flag = NonReentrantMutex::new(false);

        flag.with(|_f| {
            flag.with(|f| {
                *f = true;
            });
        });
    }

    #[test]
    fn priority_lock_tests(peripherals: Peripherals) {
        static COUNTER: AtomicU32 = AtomicU32::new(0);

        extern "C" fn increment<const INT: u8>() {
            unsafe { SoftwareInterrupt::<INT>::steal().reset() };
            COUNTER.fetch_add(1, Ordering::AcqRel);
        }

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let mut prio_1_interrupt = sw_ints.software_interrupt0;
        let mut prio_2_interrupt = sw_ints.software_interrupt1;

        prio_1_interrupt
            .set_interrupt_handler(InterruptHandler::new(increment::<0>, Priority::Priority1));
        prio_2_interrupt
            .set_interrupt_handler(InterruptHandler::new(increment::<1>, Priority::Priority2));

        let lock = RawPriorityLimitedMutex::new(Priority::Priority1);

        let delay = Delay::new();

        // Lock does nothing unless taken

        prio_1_interrupt.raise();
        // Software interrupts may not trigger immediately and there may be some
        // instructions executed after `raise`. We need to wait a short while
        // to ensure that the interrupt has been serviced before reading the counter.
        delay.delay_millis(1);
        assert_eq!(COUNTER.load(Ordering::Acquire), 1);

        // Taking the lock masks the lower priority interrupt
        lock.lock(|| {
            prio_1_interrupt.raise();
            delay.delay_millis(1);
            assert_eq!(COUNTER.load(Ordering::Acquire), 1); // not incremented

            // Taken lock does not mask higher priority interrupts
            prio_2_interrupt.raise();
            delay.delay_millis(1);
            assert_eq!(COUNTER.load(Ordering::Acquire), 2);
        });

        // Releasing the lock unmasks the lower priority interrupt
        delay.delay_millis(1);
        assert_eq!(COUNTER.load(Ordering::Acquire), 3);
    }

    #[test]
    fn priority_lock_allows_access_from_equal_priority(peripherals: Peripherals) {
        test_access_at_priority(peripherals, Priority::Priority1);
    }

    #[test]
    #[should_panic]
    fn priority_lock_panics_on_higher_priority_access(peripherals: Peripherals) {
        test_access_at_priority(peripherals, Priority::Priority2);
    }

    #[test]
    fn max_priority_lock_is_masking_interrupt(peripherals: Peripherals) {
        static COUNTER: AtomicU32 = AtomicU32::new(0);

        extern "C" fn increment<const INT: u8>() {
            unsafe { SoftwareInterrupt::<INT>::steal().reset() };
            COUNTER.fetch_add(1, Ordering::AcqRel);
        }

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let mut interrupt = sw_ints.software_interrupt0;
        interrupt.set_interrupt_handler(InterruptHandler::new(increment::<0>, Priority::Priority1));

        let lock = RawPriorityLimitedMutex::new(Priority::max());

        let delay = Delay::new();

        // Taking the lock masks the lower priority interrupt
        lock.lock(|| {
            interrupt.raise();
            delay.delay_millis(1);
            assert_eq!(COUNTER.load(Ordering::Acquire), 0); // not incremented
        });

        // Releasing the lock unmasks the lower priority interrupt
        delay.delay_millis(1);
        assert_eq!(COUNTER.load(Ordering::Acquire), 1);
    }

    #[test]
    #[cfg(multi_core)]
    fn critical_section_on_multi_core(p: Peripherals) {
        // TODO: test other locks, too
        use core::{cell::Cell, sync::atomic::AtomicBool};

        use critical_section::Mutex;
        use esp_hal::system::{CpuControl, Stack};
        use hil_test::mk_static;

        static COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
        static START_COUNTING: AtomicBool = AtomicBool::new(false);
        static DONE_COUNTING: AtomicBool = AtomicBool::new(false);

        let mut cpu_control = CpuControl::new(p.CPU_CTRL);
        let app_core_stack = mk_static!(Stack<8192>, Stack::new());

        let cpu1_fnctn = || {
            while !START_COUNTING.load(Ordering::Relaxed) {}
            for _ in 0..1000 {
                critical_section::with(|cs| {
                    let data_ref = COUNTER.borrow(cs);
                    data_ref.set(data_ref.get() + 1);
                });
            }
            DONE_COUNTING.store(true, Ordering::Relaxed);
            loop {}
        };

        let _guard = cpu_control
            .start_app_core(app_core_stack, cpu1_fnctn)
            .unwrap();

        START_COUNTING.store(true, Ordering::Relaxed);
        for _ in 0..1000 {
            critical_section::with(|cs| {
                let data_ref = COUNTER.borrow(cs);
                data_ref.set(data_ref.get() + 2);
            });
        }

        while !DONE_COUNTING.load(Ordering::Relaxed) {}

        critical_section::with(|cs| {
            let data_ref = COUNTER.borrow(cs);
            assert_eq!(data_ref.get(), 3000);
        });
    }
}

#[embedded_test::tests(default_timeout = 2, executor = hil_test::Executor::new())]

mod delay_async {
    use embedded_hal_async::delay::DelayNs;
    #[cfg(systimer)]
    use esp_hal::timer::systimer::SystemTimer;
    use esp_hal::{
        peripherals::Peripherals,
        timer::{OneShotTimer, timg::TimerGroup},
    };
    struct Context {
        peripherals: Peripherals,
    }

    async fn test_async_delay_ns(mut timer: impl DelayNs, duration: u32) {
        for i in 1..5 {
            let t1 = esp_hal::time::Instant::now();
            timer.delay_ns(duration).await;
            let t2 = esp_hal::time::Instant::now();

            assert!(t2 > t1);
            assert!(
                (t2 - t1).as_micros() >= duration.div_ceil(1000) as u64,
                "diff[{}]: {:?} >= {}",
                i,
                (t2 - t1).as_micros(),
                duration
            );
        }
    }

    async fn test_async_delay_us(mut timer: impl DelayNs, duration: u32) {
        for _ in 1..5 {
            let t1 = esp_hal::time::Instant::now();
            timer.delay_us(duration).await;
            let t2 = esp_hal::time::Instant::now();

            assert!(t2 > t1);
            assert!(
                (t2 - t1).as_micros() >= duration as u64,
                "diff: {:?}",
                (t2 - t1).as_micros()
            );
        }
    }

    async fn test_async_delay_ms(mut timer: impl DelayNs, duration: u32) {
        for _ in 1..5 {
            let t1 = esp_hal::time::Instant::now();
            timer.delay_ms(duration).await;
            let t2 = esp_hal::time::Instant::now();

            assert!(t2 > t1);
            assert!(
                (t2 - t1).as_millis() >= duration as u64,
                "diff: {:?}",
                (t2 - t1).as_millis()
            );
        }
    }
    #[init]
    fn init() -> Context {
        Context {
            peripherals: esp_hal::init(esp_hal::Config::default()),
        }
    }

    #[cfg(systimer)]
    #[test]
    async fn test_systimer_async_delay_ns(ctx: Context) {
        let alarms = SystemTimer::new(ctx.peripherals.SYSTIMER);

        test_async_delay_ns(OneShotTimer::new(alarms.alarm0).into_async(), 10_000).await;
    }

    #[cfg(timergroup_timg0)]
    #[test]
    async fn test_timg0_async_delay_ns(ctx: Context) {
        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);

        test_async_delay_ns(OneShotTimer::new(timg0.timer0).into_async(), 10_000).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_ns(OneShotTimer::new(timg0.timer1).into_async(), 10_000).await;
    }

    #[cfg(timergroup_timg1)]
    #[test]
    async fn test_timg1_async_delay_ns(ctx: Context) {
        let timg1 = TimerGroup::new(ctx.peripherals.TIMG1);

        test_async_delay_ns(OneShotTimer::new(timg1.timer0).into_async(), 10_000).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_ns(OneShotTimer::new(timg1.timer1).into_async(), 10_000).await;
    }

    #[cfg(systimer)]
    #[test]
    async fn test_systimer_async_delay_us(ctx: Context) {
        let alarms = SystemTimer::new(ctx.peripherals.SYSTIMER);

        test_async_delay_us(OneShotTimer::new(alarms.alarm0).into_async(), 10).await;
    }

    #[cfg(timergroup_timg0)]
    #[test]
    async fn test_timg0_async_delay_us(ctx: Context) {
        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);

        test_async_delay_us(OneShotTimer::new(timg0.timer0).into_async(), 10).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_us(OneShotTimer::new(timg0.timer1).into_async(), 10).await;
    }

    #[cfg(timergroup_timg1)]
    #[test]
    async fn test_timg1_async_delay_us(ctx: Context) {
        let timg1 = TimerGroup::new(ctx.peripherals.TIMG1);

        test_async_delay_us(OneShotTimer::new(timg1.timer0).into_async(), 10).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_us(OneShotTimer::new(timg1.timer1).into_async(), 10).await;
    }

    #[cfg(systimer)]
    #[test]
    async fn test_systimer_async_delay_ms(ctx: Context) {
        let alarms = SystemTimer::new(ctx.peripherals.SYSTIMER);

        test_async_delay_ms(OneShotTimer::new(alarms.alarm0).into_async(), 1).await;
    }

    #[cfg(timergroup_timg0)]
    #[test]
    async fn test_timg0_async_delay_ms(ctx: Context) {
        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);

        test_async_delay_ms(OneShotTimer::new(timg0.timer0).into_async(), 1).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_ms(OneShotTimer::new(timg0.timer1).into_async(), 1).await;
    }

    #[cfg(timergroup_timg1)]
    #[test]
    async fn test_timg1_async_delay_ms(ctx: Context) {
        let timg1 = TimerGroup::new(ctx.peripherals.TIMG1);

        test_async_delay_ms(OneShotTimer::new(timg1.timer0).into_async(), 1).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_ms(OneShotTimer::new(timg1.timer1).into_async(), 1).await;
    }
}

#[embedded_test::tests(default_timeout = 3)]
mod dma_macros {
    const DATA_SIZE: usize = 1024 * 10;

    pub(crate) const fn compute_size(size: usize, chunk_size: usize) -> usize {
        size.div_ceil(chunk_size)
    }

    pub(crate) const fn compute_circular_size(size: usize, chunk_size: usize) -> usize {
        if size > chunk_size * 2 {
            size.div_ceil(chunk_size)
        } else {
            3
        }
    }
    // defmt::* is load-bearing, it ensures that the assert in dma_buffers! is not
    // using defmt's non-const assert. Doing so would result in a compile error.
    #[allow(unused_imports)]
    use defmt::*;

    #[test]
    fn test_dma_descriptors_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (rx_descriptors, tx_descriptors) = esp_hal::dma_descriptors!(DATA_SIZE);
        core::assert_eq!(rx_descriptors.len(), tx_descriptors.len());
        core::assert_eq!(rx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_descriptors_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const RX_SIZE: usize = DATA_SIZE / 2;
        const TX_SIZE: usize = DATA_SIZE;
        let (rx_descriptors, tx_descriptors) = esp_hal::dma_descriptors!(RX_SIZE, TX_SIZE);
        core::assert_eq!(rx_descriptors.len(), compute_size(RX_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(TX_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_circular_descriptors_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (rx_descriptors, tx_descriptors) = esp_hal::dma_circular_descriptors!(DATA_SIZE);
        core::assert_eq!(rx_descriptors.len(), tx_descriptors.len());
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_circular_descriptors_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const RX_SIZE: usize = DATA_SIZE / 2;
        const TX_SIZE: usize = CHUNK_SIZE * 2;
        let (rx_descriptors, tx_descriptors) = esp_hal::dma_circular_descriptors!(RX_SIZE, TX_SIZE);
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(RX_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(TX_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_buffers_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_buffers!(DATA_SIZE);
        core::assert_eq!(rx_buffer.len(), DATA_SIZE);
        core::assert_eq!(tx_buffer.len(), DATA_SIZE);
        core::assert_eq!(tx_descriptors.len(), rx_descriptors.len());
        core::assert_eq!(rx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_buffers_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const RX_SIZE: usize = DATA_SIZE / 2;
        const TX_SIZE: usize = DATA_SIZE;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_buffers!(RX_SIZE, TX_SIZE);
        core::assert_eq!(rx_buffer.len(), RX_SIZE);
        core::assert_eq!(tx_buffer.len(), TX_SIZE);
        core::assert_eq!(rx_descriptors.len(), compute_size(RX_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(TX_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_circular_buffers_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_circular_buffers!(DATA_SIZE);
        core::assert_eq!(rx_buffer.len(), DATA_SIZE);
        core::assert_eq!(tx_buffer.len(), DATA_SIZE);
        core::assert_eq!(rx_descriptors.len(), tx_descriptors.len());
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_circular_buffers_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const RX_SIZE: usize = CHUNK_SIZE * 2;
        const TX_SIZE: usize = CHUNK_SIZE * 4;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_circular_buffers!(RX_SIZE, TX_SIZE);
        core::assert_eq!(rx_buffer.len(), RX_SIZE);
        core::assert_eq!(tx_buffer.len(), TX_SIZE);
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(RX_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(TX_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_descriptors_chunk_size_same_size() {
        const CHUNK_SIZE: usize = 2048;
        let (rx_descriptors, tx_descriptors) =
            esp_hal::dma_descriptors_chunk_size!(DATA_SIZE, CHUNK_SIZE);
        core::assert_eq!(rx_descriptors.len(), tx_descriptors.len());
        core::assert_eq!(rx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_descriptors_chunk_size_different_size() {
        const CHUNK_SIZE: usize = 2048;
        const RX_SIZE: usize = DATA_SIZE / 2;
        const TX_SIZE: usize = DATA_SIZE;
        let (rx_descriptors, tx_descriptors) =
            esp_hal::dma_descriptors_chunk_size!(RX_SIZE, TX_SIZE, CHUNK_SIZE);
        core::assert_eq!(rx_descriptors.len(), compute_size(RX_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(TX_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_circular_buffers_chunk_size_same_size() {
        const CHUNK_SIZE: usize = 2048;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_circular_buffers_chunk_size!(DATA_SIZE, CHUNK_SIZE);
        core::assert_eq!(rx_buffer.len(), DATA_SIZE);
        core::assert_eq!(tx_buffer.len(), DATA_SIZE);
        core::assert_eq!(rx_descriptors.len(), tx_descriptors.len());
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_circular_buffers_chunk_size_different_size() {
        const CHUNK_SIZE: usize = 2048;
        const RX_SIZE: usize = DATA_SIZE / 2;
        const TX_SIZE: usize = DATA_SIZE;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_circular_buffers_chunk_size!(RX_SIZE, TX_SIZE, CHUNK_SIZE);
        core::assert_eq!(rx_buffer.len(), RX_SIZE);
        core::assert_eq!(tx_buffer.len(), TX_SIZE);
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(RX_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(TX_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_tx_buffer() {
        use esp_hal::dma::{DmaBufError, DmaTxBuf};
        const TX_SIZE: usize = DATA_SIZE;

        fn check(result: Result<DmaTxBuf, DmaBufError>, size: usize) {
            match result {
                Ok(tx_buf) => {
                    core::assert_eq!(tx_buf.len(), size);
                }
                Err(_) => {
                    core::panic!("Failed to create DmaTxBuf");
                }
            }
        }
        check(esp_hal::dma_tx_buffer!(TX_SIZE), TX_SIZE);
        check(esp_hal::dma_tx_buffer!(TX_SIZE + 1), TX_SIZE + 1);
        check(esp_hal::dma_tx_buffer!(TX_SIZE + 2), TX_SIZE + 2);
        check(esp_hal::dma_tx_buffer!(TX_SIZE + 3), TX_SIZE + 3);
    }
}

#[cfg(not(esp32))]
#[embedded_test::tests(default_timeout = 3)]
mod dma_mem2mem {
    use esp_hal::{
        Blocking,
        dma::{DmaError, Mem2Mem},
        dma_buffers,
        dma_descriptors,
    };
    const DATA_SIZE: usize = 1024 * 10;

    struct Context {
        mem2mem: Mem2Mem<'static, Blocking>,
    }
    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                let mem2mem = Mem2Mem::new(peripherals.DMA_COPY);
            } else if #[cfg(any(esp32c2, esp32c6, esp32h2))] {
                let mem2mem = Mem2Mem::new(peripherals.DMA_CH0, peripherals.MEM2MEM1);
            } else {
                let mem2mem = Mem2Mem::new(peripherals.DMA_CH0, peripherals.SPI2);
            }
        }

        Context { mem2mem }
    }

    #[test]
    fn test_internal_mem2mem(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DATA_SIZE);

        let mut mem2mem = ctx
            .mem2mem
            .with_descriptors(rx_descriptors, tx_descriptors, Default::default())
            .unwrap();

        for i in 0..core::mem::size_of_val(tx_buffer) {
            tx_buffer[i] = (i % 256) as u8;
        }
        let dma_wait = mem2mem.start_transfer(rx_buffer, tx_buffer).unwrap();
        dma_wait.wait().unwrap();
        for i in 0..core::mem::size_of_val(tx_buffer) {
            assert_eq!(rx_buffer[i], tx_buffer[i]);
        }
    }

    #[test]
    fn test_mem2mem_errors_zero_tx(ctx: Context) {
        let (rx_descriptors, tx_descriptors) = dma_descriptors!(1024, 0);
        match ctx
            .mem2mem
            .with_descriptors(rx_descriptors, tx_descriptors, Default::default())
        {
            Err(DmaError::OutOfDescriptors) => (),
            _ => panic!("Expected OutOfDescriptors"),
        }
    }

    #[test]
    fn test_mem2mem_errors_zero_rx(ctx: Context) {
        let (rx_descriptors, tx_descriptors) = dma_descriptors!(0, 1024);
        match ctx
            .mem2mem
            .with_descriptors(rx_descriptors, tx_descriptors, Default::default())
        {
            Err(DmaError::OutOfDescriptors) => (),
            _ => panic!("Expected OutOfDescriptors"),
        }
    }
}

#[embedded_test::tests(default_timeout = 3)]
mod init {
    use esp_hal::{
        Config,
        clock::CpuClock,
        config::{WatchdogConfig, WatchdogStatus},
        delay::Delay,
        rtc_cntl::Rtc,
        time::Duration,
        timer::timg::TimerGroup,
    };
    #[test]
    #[cfg(timergroup_timg0)]
    fn test_feeding_timg0_wdt() {
        let peripherals = esp_hal::init(
            Config::default().with_watchdog(
                WatchdogConfig::default()
                    .with_timg0(WatchdogStatus::Enabled(Duration::from_millis(500))),
            ),
        );

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let mut wdt0 = timg0.wdt;
        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            wdt0.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        wdt0.disable();
    }

    #[test]
    #[cfg(timergroup_timg0)]
    fn test_wdt0_uses_prescaler() {
        let p = esp_hal::init(
            Config::default().with_watchdog(
                WatchdogConfig::default()
                    .with_timg0(WatchdogStatus::Enabled(Duration::from_micros(53_687_092))), // multiplied by 80 (for the default clock source), then taking the 32 lower bits this is 0x40
            ),
        );

        let delay = Delay::new();
        delay.delay(Duration::from_millis(250));

        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        let timg0 = TimerGroup::new(p.TIMG0);
        let mut wdt0 = timg0.wdt;
        wdt0.disable();
    }

    #[test]
    #[cfg(timergroup_timg1)]
    fn test_feeding_timg1_wdt() {
        let peripherals = esp_hal::init(
            Config::default().with_watchdog(
                WatchdogConfig::default()
                    .with_timg1(WatchdogStatus::Enabled(Duration::from_millis(500))),
            ),
        );

        let timg1 = TimerGroup::new(peripherals.TIMG1);
        let mut wdt1 = timg1.wdt;
        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            wdt1.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        wdt1.disable();
    }

    #[test]
    #[cfg(timergroup_timg0)]
    fn test_feeding_timg0_wdt_max_clock() {
        let peripherals = esp_hal::init(
            Config::default()
                .with_cpu_clock(CpuClock::max())
                .with_watchdog(
                    WatchdogConfig::default()
                        .with_timg0(WatchdogStatus::Enabled(Duration::from_millis(500))),
                ),
        );

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let mut wdt0 = timg0.wdt;
        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            wdt0.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        wdt0.disable();
    }

    #[test]
    fn test_feeding_rtc_wdt() {
        let peripherals = esp_hal::init(
            Config::default().with_watchdog(
                WatchdogConfig::default()
                    .with_rwdt(WatchdogStatus::Enabled(Duration::from_millis(500))),
            ),
        );

        let mut rtc = Rtc::new(peripherals.LPWR);
        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            rtc.rwdt.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        rtc.rwdt.disable();
    }

    #[test]
    fn test_default_config() {
        esp_hal::init(Config::default());

        let delay = Delay::new();
        delay.delay(Duration::from_millis(1000));
    }
}
