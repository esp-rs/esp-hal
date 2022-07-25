#![no_std]
#![no_main]

use embedded_graphics::prelude::RgbColor;
use esp32s2_hal::{clock::ClockControl, pac::Peripherals, prelude::*, timer::TimerGroup, RtcCntl};
use esp_println::println;
use esp_backtrace as _;
use xtensa_atomic_emulation_trap as _;
use xtensa_lx_rt::entry;

use esp32s2_hal::{gpio};
use esp32s2_hal::spi;
use esp32s2_hal::prelude::*;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let mut clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc_cntl.set_wdt_global_enable(false);
    wdt0.disable();
    wdt1.disable();


    println!("About to initialize the ESP32-S2/S3-USB-OTG SPI LED driver ST7789VW");
    use esp32s2_hal::IO;
    // let mut system = peripherals.SYSTEM.split();
    // let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let backlight= io.pins.gpio9;
    let mut backlight = backlight.into_push_pull_output();
    backlight.set_high();
    use embedded_graphics::draw_target::DrawTarget;


//     dc: gpio::Gpio4<gpio::Pins>,
//     rst: gpio::Gpio8<gpio::Pins>,
//     peripheral_spi: spi::Spi<spi::SpiMode>,
//     sclk: gpio::Gpio6<gpio::Pins>,
//     mosi: gpio::Gpio7<gpio::Pins>,
//     cs: gpio::Gpio5<gpio::Pins>,
//     peripheral_clock_control:ClockControl,
let mosi = io.pins.gpio7;
let cs = io.pins.gpio5;
let rst = io.pins.gpio8;
let dc = io.pins.gpio4;
    // let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
     let mut spi = spi::Spi::new(
      peripherals.SPI3,
      io.pins.gpio6,
     mosi,
     Some(io.pins.gpio12),
     Some(cs),
     80u32.kHz(),
     spi::SpiMode::Mode0,
     &mut system.peripheral_clock_control,
     &mut clocks,
 );
 use display_interface_spi::SPIInterfaceNoCS;
 let di = SPIInterfaceNoCS::new(spi, dc.into_push_pull_output());
    // let di = SPIInterfaceNoCS::new(
    //     spi::Master::<spi::SPI3, _, _, _, _>::new(
    //         spi,
    //         spi::Pins {
    //             sclk,
    //             sdo,
    //             sdi: Option::<gpio::Gpio21<gpio::Unknown>>::None,
    //             cs: Some(cs),
    //         },
    //         config,
    //     )?,
    //     dc.into_output()?,
    // );

    let reset = rst.into_push_pull_output();

    let mut display = st7789::ST7789::new(di, reset, 240, 240);
    use esp32s2_hal::{Delay};
    let mut delay = Delay::new(&clocks);
    display.init(&mut delay).unwrap();
      // .map_err(|err| error!("{:?}", err))
      // .err();
    display
        .set_orientation(st7789::Orientation::Landscape);
        // .map_err(|err| error!("{:?}", err))
        // .ok();
        display.clear(RgbColor::WHITE);
        use embedded_graphics::{text::Text,   mono_font::{
          ascii::{FONT_6X10, FONT_9X18_BOLD},
          MonoTextStyleBuilder,
      }, };
      use embedded_graphics::mono_font::{ascii::FONT_8X13, MonoTextStyle};
      use embedded_graphics::prelude::Point;
      use embedded_graphics::Drawable;
        Text::new(
          "Bare Metal is for everyone! ",
          Point::new(10, 110),
          MonoTextStyle::new(&FONT_8X13, RgbColor::RED)
      )
      .draw(&mut display).unwrap();

      
      Text::new(
        "160-165 BPM",
        Point::new(100, 140),
        MonoTextStyle::new(&FONT_8X13, RgbColor::GREEN)
    )
    .draw(&mut display).unwrap();
        println!("Initialized");

        loop {}
}
