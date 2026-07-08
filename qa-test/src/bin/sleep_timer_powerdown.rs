//! Timer-woken light sleep with the CPU/TOP power domains powered down.
//!
//! Three policies for the same timer wakeup: clock-gated (plain light sleep),
//! cpu-powerdown (CPU state saved in software) and top-powerdown (whole `TOP`
//! domain off via regDMA, also powering the CPU down). Each round prints the
//! time slept and `cpu_power_down_wake_count`, which advances only when the CPU
//! lost power.
//!
//! It also proves the design end to end:
//!
//! - **Negative control**: a sentinel stamped into an un-retained `TOP` register (I2C0
//!   `SCL_LOW_PERIOD`) survives a clock-gated sleep, but a `top-powerdown` (after the driver is
//!   dropped) wipes it and the CPU counter advances - the domain lost power. The I2C0 clock is
//!   briefly re-enabled afterwards so the register can be read back validly.
//! - **Safety net**: an active, un-retained UART1 holds a `TOP` lock, so `top-powerdown` degrades
//!   to clock-gating (the counter stays put).
//! - **Opt-in retention**: UART1/I2C0/SPI2 are retained and a config register of each is confirmed
//!   to survive `top-powerdown`.
//!
//! GPIO5 is high while awake, low while asleep, to bracket each sleep on a
//! current meter / logic analyzer.

//% CHIP_FILTER: esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c, I2cRetentionMemory},
    main,
    peripherals::SYSTEM,
    rtc_cntl::{
        Rtc,
        cpu_retention::{CpuRetentionMemory, SystemRetentionMemory, cpu_power_down_wake_count},
        sleep::{LowPower, RtcSleepConfig, TimerWakeupSource},
    },
    spi::master::{Config as SpiConfig, Spi, SpiRetentionMemory},
    time::Duration,
    uart::{Config as UartConfig, Uart, UartRetentionMemory},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

/// Allocate `$val` in a `static` and hand out a `&'static mut` to it.
macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.uninit().write($val)
    }};
}

/// Sleep duration per round (ms), wide enough to average on a current meter.
const EVENT_MS: u64 = 1000;
/// Awake window between sleeps (ms), to separate the plateaus on a trace.
const AWAKE_MS: u32 = 200;
/// Rounds per mode.
const ROUNDS: u32 = 3;

/// Sleep once for `EVENT_MS` and report the time slept (from the RTC) and the
/// CPU power-down count. `marker` is driven low for the sleep window.
fn sleep_round(
    rtc: &mut Rtc<'_>,
    lpwr: &mut LowPower<'_>,
    marker: &mut Output<'_>,
    delay: &Delay,
    config: &RtcSleepConfig,
    label: &str,
    round: u32,
) {
    let timer = TimerWakeupSource::new(Duration::from_millis(EVENT_MS));

    let rtc_before = rtc.time_since_power_up().as_micros();
    marker.set_low();
    lpwr.sleep(config, &[&timer]);
    marker.set_high();
    let slept_ms = (rtc.time_since_power_up().as_micros() - rtc_before) / 1000;

    println!(
        "{} #{}: slept ~{} ms (RTC), CPU power-downs = {}",
        label,
        round,
        slept_ms,
        cpu_power_down_wake_count()
    );

    delay.delay_millis(AWAKE_MS);
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut rtc = Rtc::new(peripherals.RTC_TIMER);
    let mut lpwr = LowPower::new(peripherals.LPWR);
    let delay = Delay::new();

    // Awake = high, asleep = low. The IO domain stays powered, so the pin holds.
    let mut marker = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());

    // Each power-down config gets its own caller-owned retention buffer.
    let clock_gated = RtcSleepConfig::default();
    let cpu_pd = RtcSleepConfig::default()
        .with_cpu_power_down(mk_static!(CpuRetentionMemory, CpuRetentionMemory::new()));
    let top_pd = RtcSleepConfig::default().with_top_power_down(
        mk_static!(CpuRetentionMemory, CpuRetentionMemory::new()),
        mk_static!(SystemRetentionMemory, SystemRetentionMemory::new()),
    );

    println!("up and running!");

    // Negative control: I2C0's SCL_LOW_PERIOD (a TOP register) survives a
    // clock-gated sleep, but once its driver is dropped (releasing its TOP lock)
    // a top-powerdown wipes it - proof the domain lost power.
    const I2C0_SCL_LOW_PERIOD: *const u32 = 0x6000_4000 as *const u32;
    let i2c0 = I2c::new(peripherals.I2C0, I2cConfig::default()).unwrap();
    // Stamp a recognizable sentinel into the TOP register so its fate is obvious
    // in the log; the exact value doesn't matter, only whether it survives.
    const SENTINEL: u32 = 0x155;
    // SAFETY: driver alive, register writable and readable.
    let scl_configured = unsafe {
        (I2C0_SCL_LOW_PERIOD as *mut u32).write_volatile(SENTINEL);
        I2C0_SCL_LOW_PERIOD.read_volatile()
    };

    marker.set_low();
    lpwr.sleep(
        &clock_gated,
        &[&TimerWakeupSource::new(Duration::from_millis(EVENT_MS))],
    );
    marker.set_high();
    // Read while the driver is still alive (TOP stayed powered, so it survives).
    let scl_after_clock_gate = unsafe { I2C0_SCL_LOW_PERIOD.read_volatile() };

    // Drop the driver so its TOP lock is released and the domain can power down.
    // This also gates the I2C0 clock, so its registers can't be read back until
    // the clock is turned on again after the sleep.
    core::mem::drop(i2c0);

    let downs_before_nc = cpu_power_down_wake_count();
    marker.set_low();
    lpwr.sleep(
        &top_pd,
        &[&TimerWakeupSource::new(Duration::from_millis(EVENT_MS))],
    );
    marker.set_high();
    let downs_after_nc = cpu_power_down_wake_count();

    // Re-enable just the I2C0 clock (no reset) before reading. With the clock
    // gated the bus reads 0 regardless of retention, which would prove nothing;
    // with it on, a reset value here means TOP genuinely lost the register's
    // contents across the power-down.
    SYSTEM::regs()
        .i2c0_conf()
        .modify(|_, w| w.i2c0_clk_en().set_bit());
    // SAFETY: register readable now the clock is on; reads its reset value (0)
    // because TOP lost power and reset it.
    let scl_after_top_pd = unsafe { I2C0_SCL_LOW_PERIOD.read_volatile() };

    println!(
        "negative-control I2C0 SCL_LOW_PERIOD: configured = {:#x}",
        scl_configured
    );
    println!(
        "  after clock-gated (== no-retention/main behavior): {:#x} -> {}",
        scl_after_clock_gate,
        if scl_after_clock_gate == scl_configured {
            "SURVIVED (TOP stayed powered)"
        } else {
            "CHANGED?!"
        }
    );
    println!(
        "  after top-powerdown (no retention): {:#x} -> {} (CPU power-downs {} -> {})",
        scl_after_top_pd,
        if scl_after_top_pd != scl_configured && scl_after_top_pd == 0 {
            "WIPED (TOP lost power) -> POWER REALLY REMOVED"
        } else {
            "STILL SET?!"
        },
        downs_before_nc,
        downs_after_nc
    );

    // UART1 is live but not retained, so it holds a TOP lock: top-powerdown must
    // degrade to clock-gating (counter must not advance).
    const UART1_CLKDIV: *const u32 = 0x6000_1014 as *const u32;
    let uart1 = Uart::new(peripherals.UART1, UartConfig::default()).unwrap();

    let downs_before_block = cpu_power_down_wake_count();
    for round in 1..=2 {
        sleep_round(
            &mut rtc,
            &mut lpwr,
            &mut marker,
            &delay,
            &top_pd,
            "top-blocked  ",
            round,
        );
    }
    let downs_after_block = cpu_power_down_wake_count();
    println!(
        "safety-net: uart1 active + un-retained -> top-powerdown degraded, CPU power-downs {} (was {}) -> {}",
        downs_after_block,
        downs_before_block,
        if downs_after_block == downs_before_block {
            "BLOCKED (clock-gated) -> SAFE"
        } else {
            "POWERED DOWN -> UNSAFE"
        }
    );

    // Retain UART1: drops the lock so its registers are saved/restored around the
    // power-down. Kept alive for the proof.
    let _uart1 =
        uart1.with_retention_memory(mk_static!(UartRetentionMemory, UartRetentionMemory::new()));
    // SAFETY: driver alive, register readable.
    let clkdiv_before = unsafe { UART1_CLKDIV.read_volatile() };

    // Re-acquire I2C0 (the negative control dropped it) and retain it too.
    // SAFETY: the earlier I2C0 driver was dropped, so no other instance is live.
    let i2c0 = I2c::new(
        unsafe { esp_hal::peripherals::I2C0::steal() },
        I2cConfig::default(),
    )
    .unwrap();
    let _i2c0 =
        i2c0.with_retention_memory(mk_static!(I2cRetentionMemory, I2cRetentionMemory::new()));
    // SAFETY: driver alive, register readable.
    let i2c_scl_before = unsafe { I2C0_SCL_LOW_PERIOD.read_volatile() };

    // ...and SPI2, whose CLOCK register holds the divider from `Spi::new`.
    const SPI2_CLOCK: *const u32 = 0x6008_100C as *const u32;
    let spi2 = Spi::new(peripherals.SPI2, SpiConfig::default()).unwrap();
    let _spi2 =
        spi2.with_retention_memory(mk_static!(SpiRetentionMemory, SpiRetentionMemory::new()));
    // SAFETY: driver alive, register readable.
    let spi_clock_before = unsafe { SPI2_CLOCK.read_volatile() };

    // Same timer wakeup, increasingly aggressive power policies.
    let modes: [(&str, RtcSleepConfig); 3] = [
        ("clock-gated  ", clock_gated),
        ("cpu-powerdown", cpu_pd),
        ("top-powerdown", top_pd),
    ];

    for (label, config) in &modes {
        for round in 1..=ROUNDS {
            sleep_round(
                &mut rtc,
                &mut lpwr,
                &mut marker,
                &delay,
                config,
                label,
                round,
            );
        }
    }

    // With retention, UART1's CLKDIV survives the power-down.
    let clkdiv_after = unsafe { UART1_CLKDIV.read_volatile() };
    println!(
        "UART1 CLKDIV: before = {:#x}, after top-powerdown = {:#x} -> {}",
        clkdiv_before,
        clkdiv_after,
        if clkdiv_after == clkdiv_before && clkdiv_after != 0 {
            "RETAINED"
        } else {
            "LOST"
        }
    );

    // Same check for I2C0's timing register.
    let i2c_scl_after = unsafe { I2C0_SCL_LOW_PERIOD.read_volatile() };
    println!(
        "I2C0 SCL_LOW_PERIOD: before = {:#x}, after top-powerdown = {:#x} -> {}",
        i2c_scl_before,
        i2c_scl_after,
        if i2c_scl_after == i2c_scl_before && i2c_scl_after != 0 {
            "RETAINED"
        } else {
            "LOST"
        }
    );

    // ...and for SPI2's clock register.
    let spi_clock_after = unsafe { SPI2_CLOCK.read_volatile() };
    println!(
        "SPI2 CLOCK: before = {:#x}, after top-powerdown = {:#x} -> {}",
        spi_clock_before,
        spi_clock_after,
        if spi_clock_after == spi_clock_before && spi_clock_after != 0 {
            "RETAINED"
        } else {
            "LOST"
        }
    );

    // Keep going in the deepest mode for a steady stream of sleep windows.
    let mut round = ROUNDS;
    loop {
        round += 1;
        sleep_round(
            &mut rtc,
            &mut lpwr,
            &mut marker,
            &delay,
            &top_pd,
            "top-powerdown",
            round,
        );
    }
}
