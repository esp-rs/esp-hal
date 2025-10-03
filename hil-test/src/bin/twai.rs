//! TWAI test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

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
use hil_test as _;
use nb::block;

use crate::Priority::Priority3;

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
