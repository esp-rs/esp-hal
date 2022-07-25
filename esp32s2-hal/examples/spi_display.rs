#![no_std]
#![no_main]

use embedded_graphics::{draw_target::DrawTarget, prelude::RgbColor};
use esp32s2_hal::{
    clock::ClockControl,
    gpio,
    pac::Peripherals,
    prelude::*,
    spi,
    timer::TimerGroup,
    RtcCntl,
    IO,
};
// use esp_backtrace as _;
use esp_println::println;
use panic_halt as _;
// use xtensa_atomic_emulation_trap as _;
use xtensa_lx_rt::entry;

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

    println!("About to initialize the SPI LED driver ST7789VW");
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let backlight = io.pins.gpio9;
    let mut backlight = backlight.into_push_pull_output();
    backlight.set_high();

    let mosi = io.pins.gpio7;
    let cs = io.pins.gpio5;
    let rst = io.pins.gpio8;
    let dc = io.pins.gpio4;

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

    let reset = rst.into_push_pull_output();

    let mut display = st7789::ST7789::new(di, reset, 240, 240);
    use esp32s2_hal::Delay;
    let mut delay = Delay::new(&clocks);
    display.init(&mut delay).unwrap();
    // .map_err(|err| error!("{:?}", err))
    // .err();
    display.set_orientation(st7789::Orientation::Landscape);
    // .map_err(|err| error!("{:?}", err))
    // .ok();
    display.clear(RgbColor::WHITE);
    println!("Initialized");

    use embedded_graphics::{
        mono_font::{
            ascii::{FONT_6X10, FONT_8X13, FONT_9X18_BOLD},
            MonoTextStyle,
            MonoTextStyleBuilder,
        },
        prelude::Point,
        text::Text,
        Drawable,
    };

    Text::new(
        "Hello from ESP-RS! ",
        Point::new(10, 110),
        MonoTextStyle::new(&FONT_8X13, RgbColor::BLACK),
    )
    .draw(&mut display)
    .unwrap();

    loop {}
}
