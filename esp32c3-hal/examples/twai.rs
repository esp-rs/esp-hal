#![no_std]
#![no_main]

use core::fmt::Write;
use esp32c3_hal::{
    clock::{ClockControl, CpuClock},
    gpio::IO,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    twai, Rtc, UsbSerialJtag,
};

use embedded_hal::can::{Can, Frame, Id::Standard, StandardId};
use esp_backtrace as _;
use nb::{block, Error};
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    // let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    timer0.start(500u64.millis());

    writeln!(
        UsbSerialJtag,
        "Clocks: apb: {} cpu: {} xtal: {}",
        clocks.apb_clock, clocks.cpu_clock, clocks.xtal_clock
    )
    .unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let can_config = twai::TWAIConfiguration::new(
        peripherals.TWAI,
        io.pins.gpio2,
        io.pins.gpio3,
        &mut system.peripheral_clock_control,
        twai::BaudRate::B1000K,
    );

    let mut can = can_config.start();

    let mut packets_sent = 0u64;

    loop {
        let d = packets_sent as u8;
        let data = [d, 1, 2, 3, 4, 5, 6, 7];

        let id = (packets_sent as u16) & 0x7FF;

        let mut frame =
            twai::ESPTWAIFrame::new(Standard(StandardId::new(id).unwrap()), &data).unwrap();

        let result = block!(can.transmit(&mut frame));

        // match result {
        //     Err(err) => match err {
        //         Error::WouldBlock => {
        //             writeln!(UsbSerialJtag, "TWAI would block!").unwrap();
        //         }
        //         Error::Other(err) => {
        //             writeln!(UsbSerialJtag, "TWAI hit a different error: {:?}", err).unwrap();
        //         }
        //     },
        //     _ => {}
        // }

        if let Err(err) = result {
            writeln!(UsbSerialJtag, "TWAI hit a different error: {:?}", err).unwrap();
        } else {
            packets_sent += 1;
            writeln!(UsbSerialJtag, "Sent. {}", packets_sent).unwrap();
        }

        // let status = can.status();
        // writeln!(
        //     UsbSerialJtag,
        //     "CAN Status: {:b}\n\t RX_BUF_ST {}\n\t OVERRUN_ST {}\n\t TX_BUF_ST {}\n\t TX_COMPLETE {}\n\t RX_ST {}\n\t TX_ST {}\n\t ERR_ST {}\n\t BUS_OFF_ST {}\n\t MISS_ST {}",
        //     status,
        //     (status >> 0) & 0b1 != 0,
        //     (status >> 1) & 0b1 != 0,
        //     (status >> 2) & 0b1 != 0,
        //     (status >> 3) & 0b1 != 0,
        //     (status >> 4) & 0b1 != 0,
        //     (status >> 5) & 0b1 != 0,
        //     (status >> 6) & 0b1 != 0,
        //     (status >> 7) & 0b1 != 0,
        //     (status >> 8) & 0b1 != 0,
        // )
        // .ok();

        // block!(timer0.wait()).unwrap();
    }
}
