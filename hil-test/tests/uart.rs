//! UART Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    Blocking,
    delay::Delay,
    gpio::AnyPin,
    uart::{self, ClockSource, Uart},
};
use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

struct Context {
    uart0: Uart<'static, Blocking>,
    uart1: Uart<'static, Blocking>,
    rx: AnyPin<'static>,
    tx: AnyPin<'static>,
    rts: AnyPin<'static>,
    delay: Delay,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::gpio::Pin;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );

        let (rx, tx) = hil_test::common_test_pins!(peripherals);
        let rts = hil_test::unconnected_pin!(peripherals);

        let delay = Delay::new();

        let uart0 = Uart::new(peripherals.UART0, uart::Config::default()).unwrap();
        let uart1 = Uart::new(peripherals.UART1, uart::Config::default()).unwrap();

        Context {
            uart0,
            uart1,
            rx: rx.degrade(),
            tx: tx.degrade(),
            rts: rts.degrade(),
            delay: delay,
        }
    }

    #[test]
    fn test_send_receive(ctx: Context) {
        let mut uart = ctx.uart1.with_tx(ctx.tx).with_rx(ctx.rx);

        uart.write(&[0x42]).unwrap();
        let mut byte = [0u8; 1];
        uart.read(&mut byte).unwrap();
        assert_eq!(byte[0], 0x42);
    }

    #[test]
    fn flush_waits_for_data_to_be_transmitted(ctx: Context) {
        let mut uart = ctx.uart1.with_tx(ctx.tx).with_rx(ctx.rx);

        assert!(uart.write_ready());
        assert!(!uart.read_ready());

        let bauds = [1000, 5000000];
        for baud in bauds {
            uart.apply_config(&uart::Config::default().with_baudrate(baud))
                .unwrap();
            for i in 0..10 {
                let mut byte = [0u8; 1];
                uart.write(&[i as u8]).unwrap();
                uart.flush().unwrap();

                assert!(uart.write_ready());
                assert!(uart.read_ready());

                let read = uart.read_buffered(&mut byte).unwrap();

                assert!(!uart.read_ready());
                assert_eq!(read, 1, "Baud rate {}, iteration {}", baud, i);
                assert_eq!(byte[0], i as u8, "Baud rate {}, iteration {}", baud, i);
            }
        }
    }

    #[test]
    fn test_different_tolerance(ctx: Context) {
        let mut uart = ctx.uart1.with_tx(ctx.tx).with_rx(ctx.rx);

        let configs = [
            uart::Config::default()
                .with_baudrate(19_200)
                .with_baudrate_tolerance(uart::BaudrateTolerance::Exact),
            uart::Config::default()
                .with_baudrate(9600)
                .with_baudrate_tolerance(uart::BaudrateTolerance::ErrorPercent(10)),
        ];

        for config in configs {
            uart.apply_config(&config).unwrap();

            uart.write(&[0x42]).unwrap();
            let mut byte = [0u8; 1];
            uart.read(&mut byte).unwrap();
            assert_eq!(byte[0], 0x42);
        }
    }

    #[test]
    fn test_hw_flow_control(ctx: Context) {
        let (rts_input, rts_output) = unsafe { ctx.rts.split() };
        let mut uart = ctx
            .uart1
            .with_tx(ctx.tx)
            .with_rx(ctx.rx)
            .with_rts(rts_output);

        uart.apply_config(&uart::Config::default().with_hw_flow_ctrl(
            esp_hal::uart::HwFlowControl {
                cts: uart::CtsConfig::Disabled,
                rts: uart::RtsConfig::Enabled(4),
            },
        ))
        .unwrap();

        let data: [u8; 10] = [0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9];

        // This will make RTS pin go high, as we write more than the RTS threshold.
        uart.write(&data).unwrap();
        // Required, otherwise we'll run into a timing issue and won't be able to
        // determine that the RTS is at a high level.
        ctx.delay.delay_millis(1);

        // RTS pin should go high when the threshold is exceeded.
        assert_eq!(rts_input.is_input_high(), true);
    }

    #[test]
    fn test_send_receive_buffer(ctx: Context) {
        let mut uart = ctx.uart1.with_tx(ctx.tx).with_rx(ctx.rx);

        const BUF_SIZE: usize = 128; // UART_FIFO_SIZE

        let data = [13; BUF_SIZE];
        let written = uart.write(&data).unwrap();
        assert_eq!(written, BUF_SIZE);

        // Calls to read may not fill the buffer, wait until read returns 0
        let mut buffer = [0; BUF_SIZE];
        embedded_io::Read::read_exact(&mut uart, &mut buffer).unwrap();

        assert_eq!(data, buffer);
    }

    #[test]
    fn test_send_receive_different_baud_rates_and_clock_sources(ctx: Context) {
        let mut uart = ctx.uart1.with_tx(ctx.tx).with_rx(ctx.rx);

        // The default baud rate for the UART is 115,200, so we will try to
        // send/receive with some other common baud rates to ensure this is
        // working as expected. We will also using different clock sources
        // while we're at it.
        let configs = [
            #[cfg(not(any(esp32, esp32s2)))]
            (9600, ClockSource::RcFast),
            #[cfg(not(any(esp32, esp32s2)))]
            (19_200, ClockSource::Xtal),
            #[cfg(esp32s2)]
            (9600, ClockSource::RefTick),
            (921_600, ClockSource::Apb),
            (2_000_000, ClockSource::Apb),
            (3_500_000, ClockSource::Apb),
            (5_000_000, ClockSource::Apb),
        ];

        let mut byte_to_write = 0xA5;
        for (baudrate, clock_source) in configs {
            uart.apply_config(
                &uart::Config::default()
                    .with_baudrate(baudrate)
                    .with_clock_source(clock_source),
            )
            .unwrap();
            uart.write(&[byte_to_write]).unwrap();
            let mut byte = [0u8; 1];
            uart.read(&mut byte).unwrap();

            assert_eq!(byte[0], byte_to_write);
            byte_to_write = !byte_to_write;
        }
    }

    #[test]
    fn test_send_receive_inverted(ctx: Context) {
        let mut uart = ctx
            .uart1
            .with_tx(ctx.tx.into_output_signal().with_output_inverter(true))
            .with_rx(unsafe { ctx.rx.into_input_signal() }.with_input_inverter(true));

        uart.write(&[0x42]).unwrap();
        let mut byte = [0u8; 1];
        uart.read(&mut byte).unwrap();
        assert_eq!(byte[0], 0x42);
    }

    #[test]
    fn test_split_send_receive(ctx: Context) {
        let mut tx = ctx.uart0.split().1.with_tx(ctx.tx);
        let mut rx = ctx.uart1.split().0.with_rx(ctx.rx);

        let byte = [0x42];

        tx.flush().unwrap();
        tx.write(&byte).unwrap();
        let mut buf = [0u8; 1];
        rx.read(&mut buf).unwrap();

        assert_eq!(buf[0], 0x42);
    }

    #[test]
    fn test_split_send_receive_bytes(ctx: Context) {
        let mut tx = ctx.uart0.split().1.with_tx(ctx.tx);
        let mut rx = ctx.uart1.split().0.with_rx(ctx.rx);

        let bytes = [0x42, 0x43, 0x44];
        let mut buf = [0u8; 3];

        tx.flush().unwrap();
        tx.write(&bytes).unwrap();

        embedded_io::Read::read_exact(&mut rx, &mut buf).unwrap();

        assert_eq!(buf, bytes);
    }
}
