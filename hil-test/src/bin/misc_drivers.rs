//! Interrupt Tests
//!     "Disabled" for now - see https://github.com/esp-rs/esp-hal/pull/1635#issuecomment-2137405251
//! PCNT Tests
//! RNG Tests
//! System Timer Tests
//! TWAI Tests
//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use hil_test as _;

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))]
#[embedded_test::tests(default_timeout = 3)]
mod interrupt {
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

#[cfg(not(any(esp32c2, esp32c3)))]
#[embedded_test::tests(default_timeout = 3)]
mod pcnt {
    use esp_hal::{
        delay::Delay,
        gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig, Pin, Pull},
        pcnt::{Pcnt, channel::EdgeMode},
    };

    struct Context<'d> {
        pcnt: Pcnt<'d>,
        input: AnyPin<'d>,
        output: AnyPin<'d>,
        delay: Delay,
    }
    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (din, dout) = hil_test::common_test_pins!(peripherals);

        let din = din.degrade();
        let dout = dout.degrade();

        Context {
            pcnt: Pcnt::new(peripherals.PCNT),
            input: din,
            output: dout,
            delay: Delay::new(),
        }
    }

    #[test]
    fn test_increment_on_pos_edge(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit0;

        // Setup channel 0 to increment the count when gpio2 does LOW -> HIGH
        unit.channel0.set_edge_signal(Input::new(
            ctx.input,
            InputConfig::default().with_pull(Pull::Down),
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut output = Output::new(ctx.output, Level::Low, OutputConfig::default());

        unit.resume();

        assert_eq!(0, unit.value());

        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());
    }

    #[test]
    fn test_increment_on_neg_edge(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit1;

        // Setup channel 0 to increment the count when gpio2 does LOW -> HIGH
        unit.channel0.set_edge_signal(Input::new(
            ctx.input,
            InputConfig::default().with_pull(Pull::Up),
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);

        let mut output = Output::new(ctx.output, Level::High, OutputConfig::default());

        unit.resume();

        assert_eq!(0, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());

        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());
    }

    #[test]
    fn test_increment_past_high_limit(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit3;

        unit.set_high_limit(Some(3)).unwrap();

        // Setup channel 0 to increment the count when gpio2 does LOW -> HIGH
        unit.channel0.set_edge_signal(Input::new(
            ctx.input,
            InputConfig::default().with_pull(Pull::Up),
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);

        let mut output = Output::new(ctx.output, Level::High, OutputConfig::default());

        unit.resume();

        assert_eq!(0, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(0, unit.value());
        assert!(unit.events().high_limit);
        assert!(unit.interrupt_is_set());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        // high limit event remains after next increment.
        assert!(unit.events().high_limit);

        unit.reset_interrupt();
        assert!(!unit.interrupt_is_set());

        // high limit event remains after interrupt is cleared.
        assert!(unit.events().high_limit);
    }

    #[test]
    fn test_increment_past_thresholds(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit0;

        unit.set_threshold0(Some(2));
        unit.set_threshold1(Some(4));
        // For some reason this is needed for the above thresholds to apply.
        unit.clear();

        // Setup channel 0 to increment the count when gpio2 does LOW -> HIGH
        unit.channel0.set_edge_signal(Input::new(
            ctx.input,
            InputConfig::default().with_pull(Pull::Up),
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);

        let mut output = Output::new(ctx.output, Level::High, OutputConfig::default());

        unit.resume();

        assert_eq!(0, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());
        assert!(!unit.interrupt_is_set());
        assert!(!unit.events().threshold0);
        assert!(!unit.events().threshold1);

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());
        assert!(unit.interrupt_is_set());
        assert!(unit.events().threshold0);
        assert!(!unit.events().threshold1);

        unit.reset_interrupt();

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(3, unit.value());
        assert!(!unit.interrupt_is_set());
        // threshold event remains after next increment and after interrupt is cleared.
        assert!(unit.events().threshold0);
        assert!(!unit.events().threshold1);

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(4, unit.value());
        assert!(unit.interrupt_is_set());
        // threshold event cleared after new event occurs.
        assert!(!unit.events().threshold0);
        assert!(unit.events().threshold1);
    }

    #[test]
    fn test_decrement_past_low_limit(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit0;

        unit.set_low_limit(Some(-3)).unwrap();
        // For some reason this is needed for the above limit to apply.
        unit.clear();

        // Setup channel 0 to decrement the count when gpio2 does LOW -> HIGH
        unit.channel0.set_edge_signal(Input::new(
            ctx.input,
            InputConfig::default().with_pull(Pull::Up),
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Decrement, EdgeMode::Hold);

        let mut output = Output::new(ctx.output, Level::High, OutputConfig::default());

        unit.resume();

        assert_eq!(0, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(-1, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(-2, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(0, unit.value());
        assert!(unit.events().low_limit);
        assert!(unit.interrupt_is_set());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(-1, unit.value());

        // low limit event remains after next increment.
        assert!(unit.events().low_limit);

        unit.reset_interrupt();
        assert!(!unit.interrupt_is_set());

        // low limit event remains after interrupt is cleared.
        assert!(unit.events().low_limit);
    }

    #[test]
    fn test_unit_count_range(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit2;

        // Setup channel 1 to increment the count when gpio2 does LOW -> HIGH
        unit.channel1.set_edge_signal(Input::new(
            ctx.input,
            InputConfig::default().with_pull(Pull::Up),
        ));
        unit.channel1
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);

        let mut output = Output::new(ctx.output, Level::High, OutputConfig::default());

        unit.resume();

        assert_eq!(0, unit.value());

        for i in (0..=i16::MAX).chain(i16::MIN..0) {
            assert_eq!(i, unit.value());

            output.set_low();
            ctx.delay.delay_micros(1);
            output.set_high();
            ctx.delay.delay_micros(1);
        }

        assert_eq!(0, unit.value());

        // Channel 1 should now decrement.
        unit.channel1
            .set_input_mode(EdgeMode::Decrement, EdgeMode::Hold);

        for i in (0..=i16::MAX).chain(i16::MIN..=0).rev() {
            assert_eq!(i, unit.value());

            output.set_low();
            ctx.delay.delay_micros(1);
            output.set_high();
            ctx.delay.delay_micros(1);
        }
    }
}

#[embedded_test::tests(default_timeout = 5)]
mod rng {
    use esp_hal::rng::{Rng, Trng, TrngSource};

    #[test]
    fn test_trng_without_source_is_error() {
        assert!(Trng::try_new().is_err());

        // Rng can be created anyway:
        let _rng = Rng::new();
    }

    #[test]
    fn test_trng_source_cannot_be_disabled_while_in_use() {
        let p = esp_hal::init(Default::default());
        let source = TrngSource::new(p.RNG, p.ADC1);

        let trng = Trng::try_new().unwrap();

        let _source = source.try_disable().unwrap_err();

        // Need to drop trng first
        core::mem::drop(trng);

        // Rng will not prevent disabling the TrngSource
        let _rng = Rng::new();
    }

    #[test]
    #[should_panic]
    fn test_trng_source_cannot_be_dropped_while_in_use() {
        let p = esp_hal::init(Default::default());
        let source = TrngSource::new(p.RNG, p.ADC1);

        let _trng = Trng::try_new().unwrap();

        core::mem::drop(source);
    }

    #[test]
    fn test_trng_source_can_be_dropped_if_unsafely_enabled() {
        let p = esp_hal::init(Default::default());

        let source = TrngSource::new(p.RNG, p.ADC1);

        // Unsafely increase the counter. Practically, this may be done in esp-radio.
        unsafe { TrngSource::increase_entropy_source_counter() };

        let _trng = Trng::try_new().unwrap();

        core::mem::drop(source);
    }
}

#[cfg(not(esp32))]
#[embedded_test::tests(default_timeout = 3)]
mod systimer {
    use core::cell::RefCell;

    use critical_section::Mutex;
    use embedded_hal::delay::DelayNs;
    use esp_hal::{
        Blocking,
        delay::Delay,
        handler,
        time::Duration,
        timer::{
            OneShotTimer,
            PeriodicTimer,
            systimer::{Alarm, SystemTimer},
        },
    };
    use portable_atomic::{AtomicUsize, Ordering};

    static ALARM_TARGET: Mutex<RefCell<Option<OneShotTimer<'static, Blocking>>>> =
        Mutex::new(RefCell::new(None));
    static ALARM_PERIODIC: Mutex<RefCell<Option<PeriodicTimer<'static, Blocking>>>> =
        Mutex::new(RefCell::new(None));

    struct Context {
        alarm0: Alarm<'static>,
        alarm1: Alarm<'static>,
    }

    #[handler(priority = esp_hal::interrupt::Priority::min())]
    fn pass_test_if_called() {
        critical_section::with(|cs| {
            ALARM_TARGET
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .clear_interrupt()
        });
        embedded_test::export::check_outcome(());
    }

    #[handler(priority = esp_hal::interrupt::Priority::min())]
    fn handle_periodic_interrupt() {
        critical_section::with(|cs| {
            ALARM_PERIODIC
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .clear_interrupt()
        });
    }

    static COUNTER: AtomicUsize = AtomicUsize::new(0);

    #[handler(priority = esp_hal::interrupt::Priority::min())]
    fn pass_test_if_called_twice() {
        critical_section::with(|cs| {
            ALARM_PERIODIC
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .clear_interrupt()
        });
        COUNTER.fetch_add(1, Ordering::Relaxed);
        if COUNTER.load(Ordering::Relaxed) == 2 {
            embedded_test::export::check_outcome(());
        }
    }

    #[handler(priority = esp_hal::interrupt::Priority::min())]
    fn target_fail_test_if_called_twice() {
        critical_section::with(|cs| {
            ALARM_TARGET
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .clear_interrupt()
        });
        COUNTER.fetch_add(1, Ordering::Relaxed);
        assert!(COUNTER.load(Ordering::Relaxed) != 2);
    }
    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let systimer = SystemTimer::new(peripherals.SYSTIMER);

        Context {
            alarm0: systimer.alarm0,
            alarm1: systimer.alarm1,
        }
    }

    #[test]
    fn target_interrupt_is_handled(ctx: Context) {
        let mut alarm0 = OneShotTimer::new(ctx.alarm0);

        critical_section::with(|cs| {
            alarm0.set_interrupt_handler(pass_test_if_called);
            alarm0.listen();
            alarm0.schedule(Duration::from_millis(10)).unwrap();

            ALARM_TARGET.borrow_ref_mut(cs).replace(alarm0);
        });

        // We'll end the test in the interrupt handler.
        loop {}
    }

    #[test]
    fn target_interrupt_is_handled_once(ctx: Context) {
        let mut alarm0 = OneShotTimer::new(ctx.alarm0);
        let mut alarm1 = PeriodicTimer::new(ctx.alarm1);

        COUNTER.store(0, Ordering::Relaxed);

        critical_section::with(|cs| {
            alarm0.set_interrupt_handler(target_fail_test_if_called_twice);
            alarm0.listen();
            alarm0.schedule(Duration::from_millis(10)).unwrap();

            alarm1.set_interrupt_handler(handle_periodic_interrupt);
            alarm1.listen();
            alarm1.start(Duration::from_millis(100)).unwrap();

            ALARM_TARGET.borrow_ref_mut(cs).replace(alarm0);
            ALARM_PERIODIC.borrow_ref_mut(cs).replace(alarm1);
        });

        let mut delay = Delay::new();
        delay.delay_ms(300);
    }

    #[test]
    fn periodic_interrupt_is_handled(ctx: Context) {
        let mut alarm1 = PeriodicTimer::new(ctx.alarm1);

        COUNTER.store(0, Ordering::Relaxed);

        critical_section::with(|cs| {
            alarm1.set_interrupt_handler(pass_test_if_called_twice);
            alarm1.listen();
            alarm1.start(Duration::from_millis(100)).unwrap();

            ALARM_PERIODIC.borrow_ref_mut(cs).replace(alarm1);
        });

        // We'll end the test in the interrupt handler.
        loop {}
    }
}

#[cfg(any(esp32, esp32c3, esp32c6, esp32h2, esp32s2, esp32s3))]
mod twai {
    use embedded_can::Frame;
    use esp_hal::{
        Async,
        Blocking,
        DriverMode,
        interrupt,
        interrupt::{Priority, software::SoftwareInterruptControl},
        peripherals::Interrupt::TWAI0,
        system::Cpu,
        timer::timg::TimerGroup,
        twai::{self, ErrorKind, EspTwaiFrame, StandardId, TwaiMode, filter::SingleStandardFilter},
    };
    use nb::block;

    use crate::twai::Priority::Priority3;

    struct Context<D: DriverMode> {
        twai: twai::Twai<'static, D>,
    }

    #[embedded_test::tests(default_timeout = 3)]
    mod blocking_tests {
        use super::*;

        #[init]
        fn init() -> Context<Blocking> {
            let peripherals = esp_hal::init(esp_hal::Config::default());

            let (loopback_pin, _) = hil_test::common_test_pins!(peripherals);

            let (rx, tx) = unsafe { loopback_pin.split() };

            let mut config = twai::TwaiConfiguration::new(
                peripherals.TWAI0,
                rx,
                tx,
                twai::BaudRate::B1000K,
                TwaiMode::SelfTest,
            );

            config.set_filter(SingleStandardFilter::new(
                b"00000000000",
                b"x",
                [b"xxxxxxxx", b"xxxxxxxx"],
            ));

            let twai = config.start();

            Context { twai }
        }

        #[test]
        fn test_send_receive(mut ctx: Context<Blocking>) {
            let frame = EspTwaiFrame::new_self_reception(StandardId::ZERO, &[1, 2, 3]).unwrap();
            block!(ctx.twai.transmit(&frame)).unwrap();

            let frame = block!(ctx.twai.receive()).unwrap();

            assert_eq!(frame.data(), &[1, 2, 3])
        }
    }

    #[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
    mod async_tests {
        use super::*;

        #[init]
        async fn init() -> Context<Async> {
            let peripherals = esp_hal::init(esp_hal::Config::default());

            let (loopback_pin, _) = hil_test::common_test_pins!(peripherals);

            let (rx, tx) = unsafe { loopback_pin.split() };

            let config = twai::TwaiConfiguration::new(
                peripherals.TWAI0,
                rx,
                tx,
                twai::BaudRate::B1000K,
                TwaiMode::SelfTest,
            );

            let _sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

            let timg0 = TimerGroup::new(peripherals.TIMG0);

            esp_rtos::start(
                timg0.timer0,
                #[cfg(riscv)]
                _sw_int.software_interrupt0,
            );

            let twai = config.into_async().start();

            Context { twai }
        }

        async fn transmit_frames(ctx: &mut Context<Async>, frame: &EspTwaiFrame, count: usize) {
            for _ in 0..count {
                ctx.twai.transmit_async(frame).await.unwrap();
            }
        }

        async fn receive_frames(ctx: &mut Context<Async>, expected_count: usize) {
            let mut received_count = 0;
            let mut iterations = 0;

            while received_count < expected_count {
                match ctx.twai.receive_async().await {
                    Ok(_) => received_count += 1,
                    Err(esp_hal::twai::EspTwaiError::EmbeddedHAL(ErrorKind::Overrun)) => {
                        received_count += 1;
                    }
                    Err(err) => panic!("{:#?}", err),
                }
                iterations += 1;
            }

            assert_eq!(expected_count, iterations, "receive_async loop iterations");
            assert_ne!(received_count, 0, "received_count");
        }

        #[test]
        async fn test_async_transmit_and_receive(mut ctx: Context<Async>) {
            let frame =
                EspTwaiFrame::new_self_reception(StandardId::new(0).unwrap(), b"12345678").unwrap();
            transmit_frames(&mut ctx, &frame, 31).await;
            receive_frames(&mut ctx, 31).await;
        }

        #[test]
        // regression test for https://github.com/esp-rs/esp-hal/issues/4235
        async fn test_buffer_overrun_on_empty_queue(mut ctx: Context<Async>) {
            let frame =
                EspTwaiFrame::new_self_reception(StandardId::new(0).unwrap(), b"12345678").unwrap();

            interrupt::disable(Cpu::ProCpu, TWAI0);

            const NUM_SENT_FRAMES: usize = 10;
            for _ in 0..NUM_SENT_FRAMES {
                block!(ctx.twai.transmit(&frame)).unwrap();
            }

            let _ = interrupt::enable(TWAI0, Priority3);

            const NUM_ASYNC_SENT_FRAMES: usize = 20;
            transmit_frames(&mut ctx, &frame, NUM_ASYNC_SENT_FRAMES).await;

            receive_frames(&mut ctx, NUM_SENT_FRAMES + NUM_ASYNC_SENT_FRAMES).await;
        }
    }
}
