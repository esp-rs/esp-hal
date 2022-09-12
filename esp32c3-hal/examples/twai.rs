#![no_std]
#![no_main]

use core::fmt::Write;
use esp32c3_hal::{
    clock::ClockControl,
    gpio::IO,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    twai::{
        self,
        bitselector::{BitSelectorNewExact, Selector},
        ESPTWAIFrame,
    },
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
    let filter = twai::filter::SingleStandardFilter {
        id: twai::bitselector::BitSelector {
            bits: [
                Selector::Set,
                Selector::Any,
                Selector::Any,
                Selector::Any,
                Selector::Any,
                Selector::Any,
                Selector::Any,
                Selector::Any,
                Selector::Any,
                Selector::Any,
                Selector::Any,
            ],
        },
        rtr: twai::bitselector::BitSelector::new_exact(0b0),
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

        // Increment the payload bytes by one.
        let mut data: [u8; 8] = [0; 8];
        data[..frame.dlc()].copy_from_slice(frame.data());

        for b in data[..frame.dlc()].iter_mut() {
            (*b, _) = (*b).overflowing_add(1);
        }

        let frame = ESPTWAIFrame::new(frame.id(), &data[..frame.dlc()]).unwrap();
        // Transmit the frame back.
        writeln!(UsbSerialJtag, "Transmitting: {:?}", frame).unwrap();
        let _result = block!(can.transmit(&frame)).unwrap();

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
