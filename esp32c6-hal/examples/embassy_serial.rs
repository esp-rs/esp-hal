//! embassy serial
//!
//! This is an example of running the embassy executor and asynchronously
//! writing to and reading from uart

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;

use embassy_executor::Executor;
use embassy_time::{with_timeout, Duration};
use esp32c6_hal::{
    clock::ClockControl,
    embassy,
    interrupt,
    peripherals::{Interrupt, Peripherals, UART0},
    prelude::*,
    timer::TimerGroup,
    Rtc,
    Uart,
};
use esp_backtrace as _;
use esp_hal_common::uart::config::AtCmdConfig;
use heapless::Vec;
use static_cell::StaticCell;

// rx_fifo_full_threshold
const READ_BUF_SIZE: usize = 128;
// EOT (CTRL-D)
const AT_CMD: u8 = 0x04;

#[embassy_executor::task]
async fn run(mut uart: Uart<'static, UART0>) {
    // max message size to receive
    // leave some extra space for AT-CMD characters
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;
    // timeout read
    const READ_TIMEOUT: Duration = Duration::from_secs(10);

    let mut rbuf: Vec<u8, MAX_BUFFER_SIZE> = Vec::new();
    let mut wbuf: Vec<u8, MAX_BUFFER_SIZE> = Vec::new();
    loop {
        if rbuf.is_empty() {
            embedded_hal_async::serial::Write::write(
                &mut uart,
                b"Hello async serial. Enter something ended with EOT (CTRL-D).\r\n",
            )
            .await
            .unwrap();
        } else {
            wbuf.clear();
            write!(&mut wbuf, "\r\n-- received {} bytes --\r\n", rbuf.len()).unwrap();
            embedded_hal_async::serial::Write::write(&mut uart, wbuf.as_slice())
                .await
                .unwrap();
            embedded_hal_async::serial::Write::write(&mut uart, rbuf.as_slice())
                .await
                .unwrap();
            embedded_hal_async::serial::Write::write(&mut uart, b"\r\n")
                .await
                .unwrap();
        }
        embedded_hal_async::serial::Write::flush(&mut uart)
            .await
            .unwrap();

        // set rbuf full capacity
        rbuf.resize_default(rbuf.capacity()).ok();
        let mut offset = 0;
        loop {
            match with_timeout(READ_TIMEOUT, uart.read(&mut rbuf[offset..])).await {
                Ok(r) => {
                    if let Ok(len) = r {
                        offset += len;
                        if offset == 0 {
                            rbuf.truncate(0);
                            break;
                        }
                        // if set_at_cmd is used than stop reading
                        if len < READ_BUF_SIZE {
                            rbuf.truncate(offset);
                            break;
                        }
                    } else {
                        // buffer is full
                        break;
                    }
                }
                Err(_) => {
                    // Timeout
                    rbuf.truncate(offset);
                    break;
                }
            }
        }
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32c6_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(&clocks, timer_group0.timer0);

    let mut uart0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);
    uart0.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));
    uart0.set_rx_fifo_full_threshold(READ_BUF_SIZE as u16);

    interrupt::enable(Interrupt::UART0, interrupt::Priority::Priority1).unwrap();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(run(uart0)).ok();
    });
}
