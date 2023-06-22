//! embassy serial
//!
//! This is an example of running the embassy executor and asynchronously
//! writing to a uart.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;

use embassy_executor::Executor;
use embassy_time::{with_timeout, Duration};
use esp32h2_hal::{
    clock::ClockControl,
    embassy,
    peripherals::{Interrupt, Peripherals, UART0},
    prelude::*,
    timer::TimerGroup,
    Rtc,
    Uart,
};
use esp_backtrace as _;
use esp_hal_common::uart::config::AtCmdConfig;
use static_cell::StaticCell;

struct Buffer<const N: usize> {
    len: usize,
    buf: [u8; N],
}

impl<const N: usize> Buffer<N> {
    #[inline(always)]
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    #[inline(always)]
    pub fn is_full(&self) -> bool {
        self.len == N
    }
}

impl<const N: usize> core::fmt::Write for Buffer<N> {
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let sb = s.as_bytes();
        let mut len = sb.len();
        if self.len + len > N {
            len = N - self.len;
        }
        if len > 0 {
            self.buf[self.len..self.len + len].copy_from_slice(&sb[..len]);
            self.len += len;
        }
        Ok(())
    }
}

/// rx_fifo_full_threshold
const READ_BUF_SIZE: usize = 128;
/// EOT; CTRL-D
const AT_CMD: u8 = 0x04;

#[embassy_executor::task]
async fn run(mut uart: Uart<'static, UART0>) {
    /// max READ_BUF_SIZE buffers to receive
    const MAX_BUFFERS: usize = 10;
    /// timeout read
    const READ_TIMEOUT: Duration = Duration::from_secs(5);
    let mut rbuf = Buffer {
        len: 0,
        buf: [0; MAX_BUFFERS * READ_BUF_SIZE],
    };
    let mut wbuf = Buffer {
        len: 0,
        buf: [0; 128],
    };

    loop {
        if rbuf.len == 0 {
            embedded_hal_async::serial::Write::write(
                &mut uart,
                b"Hello async serial. Enter something ended with EOT (CTRL-D).\r\n",
            )
            .await
            .unwrap();
        } else {
            wbuf.len = 0;
            write!(&mut wbuf, "\r\n-- received {} bytes --\r\n", rbuf.len).unwrap();
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

        rbuf.len = 0;
        while let Ok(Ok(len)) =
            with_timeout(READ_TIMEOUT, uart.read(&mut rbuf.buf[rbuf.len..])).await
        {
            rbuf.len += len;
            // if set_at_cmd is used than stop reading
            if rbuf.buf[rbuf.len - 1] == AT_CMD || rbuf.is_full() {
                break;
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
        esp32h2_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(&clocks, timer_group0.timer0);

    let mut uart0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);
    uart0.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));
    uart0.set_rx_fifo_full_threshold(READ_BUF_SIZE as u16);

    esp32h2_hal::interrupt::enable(Interrupt::UART0, esp32h2_hal::Priority::Priority1).unwrap();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(run(uart0)).ok();
    });
}
