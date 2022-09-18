#![no_std]
#![no_main]

use core::fmt::Write;
use esp32c3_hal::{
    clock::ClockControl,
    gpio::IO,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    twai::{self, ESPTWAIFrame},
    Rtc, UsbSerialJtag,
};

use embedded_hal::can::{Can, Frame};
use esp_backtrace as _;
use nb::block;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

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

    let mut can_config = twai::TWAIConfiguration::new(
        peripherals.TWAI,
        io.pins.gpio2,
        io.pins.gpio3,
        &mut system.peripheral_clock_control,
        &clocks,
        twai::BaudRate::B1000K,
    );

    // let filter = twai::filter::SingleStandardFilter {
    //     id: twai::filter::BitSelector::new_exact(0b10101010101),
    //     rtr: twai::filter::BitSelector::new_exact(0b0),
    //     data: [
    //         twai::filter::BitSelector::new_exact(0x24),
    //         twai::filter::BitSelector::new_any(),
    //     ],
    // };
    // TODO: even though this is a single, standard id filter, extended ids will also match this filter.
    // A filter that matches standard ids of an even value.
    let filter = twai::filter::SingleStandardFilter {
        id: twai::bitselector::BitSelector {
            bits: [
                twai::bitselector::Selector::Reset,
                twai::bitselector::Selector::Any,
                twai::bitselector::Selector::Any,
                twai::bitselector::Selector::Any,
                twai::bitselector::Selector::Any,
                twai::bitselector::Selector::Any,
                twai::bitselector::Selector::Any,
                twai::bitselector::Selector::Any,
                twai::bitselector::Selector::Any,
                twai::bitselector::Selector::Any,
                twai::bitselector::Selector::Any,
            ],
        },
        rtr: twai::bitselector::BitSelector::new_any(),
        data: [
            twai::bitselector::BitSelector::new_any(),
            twai::bitselector::BitSelector::new_any(),
        ],
    };

    // // Dump the generated regs.
    // let regs = filter.to_registers();
    // writeln!(
    //     UsbSerialJtag,
    //     "Filter Registers:\n\t{:08b} {:08b} {:08b} {:08b}\n\t{:08b} {:08b} {:08b} {:08b}",
    //     regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7],
    // )
    // .unwrap();

    can_config.set_filter(filter);

    let mut can = can_config.start();

    loop {
        // writeln!(UsbSerialJtag, "Waiting for packet...").unwrap();
        let frame = block!(can.receive()).unwrap();
        writeln!(UsbSerialJtag, "    Received: {:?}", frame).unwrap();

        let frame = if frame.is_data_frame() {
            // Increment the payload bytes by one.
            let mut data: [u8; 8] = [0; 8];
            data[..frame.dlc()].copy_from_slice(frame.data());

            for b in data[..frame.dlc()].iter_mut() {
                (*b, _) = (*b).overflowing_add(1);
            }

            ESPTWAIFrame::new(frame.id(), &data[..frame.dlc()]).unwrap()
        } else {
            // Echo back the request.
            frame
        };

        // Transmit the frame back.
        // writeln!(UsbSerialJtag, "Transmitting: {:?}", frame).unwrap();
        let _result = block!(can.transmit(&frame)).unwrap();
    }
}
