//! Uses timer0 and operator0 of the MCPWM0 peripheral to output a 50% duty signal at 20 kHz.
//!
//! The signal will be output to the pin assigned to `pin`. (GPIO4)

#![no_std]
#![no_main]

use esp32_hal::{
    clock::ClockControl,
    gpio::IO,
    mcpwm::{
        MCPWM,
        operator::PwmActions,
        timer::PwmWorkingMode,
    },
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable watchdog timer
    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let pin = io.pins.gpio4;

    // TODO check timing
    // clocks.crypto_pwm_clock is 160 MHz
    // with `prescaler = 3` pwm_clk will run at `160 MHz / (3 + 1) = 40 MHz`
    let mut mcpwm = MCPWM::new(
        peripherals.PWM0,
        &clocks,
        3,
        &mut system.peripheral_clock_control,
    );

    // with `prescaler = 19` timer0 counter will run at `40 MHz / (19 + 1) = 2 MHz`
    // with `period = 99` timer0 will run at `2 MHz / (99 + 1) = 20 kHz`
    mcpwm.timer0.start(19, 99, PwmWorkingMode::Increase);
    mcpwm.operator0.set_timer(&mcpwm.timer0);

    let mut pwm_pin = mcpwm
        .operator0
        .with_a_pin(pin, PwmActions::UP_ACTIVE_HIGH);

    // pin will be high 50% of the time
    pwm_pin.set_timestamp(50);

    loop {}
}
