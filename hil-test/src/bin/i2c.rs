//! I2C test

//% CHIP_FILTER: i2c_master_driver_supported
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Ticker};
use esp_hal::{
    Async,
    Blocking,
    i2c::master::{
        AcknowledgeCheckFailedReason,
        Config,
        Error,
        I2c,
        I2cAddress,
        Operation,
        SoftwareTimeout,
    },
    interrupt::Priority,
    peripherals::FROM_CPU_INTR2,
    time,
    timer::timg::TimerGroup,
};
use esp_rtos::embassy::InterruptExecutor;
use hil_test::mk_static;

struct Context {
    interrupt: FROM_CPU_INTR2<'static>,
    i2c: I2c<'static, Blocking>,
}

fn _async_driver_is_compatible_with_blocking_ehal() {
    fn _with_driver(driver: I2c<'static, Async>) {
        _with_ehal(driver);
    }

    fn _with_ehal(_: impl embedded_hal::i2c::I2c) {}
}

// HIL test device (MCP4725):
const DUT_ADDRESS: u8 = 0x60;
// Fast-mode write setting DAC code 0x800 (2048). On readback, bytes 1 and 2 carry
// D11:D4 = 0x80 and D3:D0 << 4 = 0x00.
const WRITE_COMMAND: &[u8] = &[0x08, 0x00];

const NON_EXISTENT_ADDRESS: u8 = 0x6b;

#[embassy_executor::task]
async fn waiting_blocking_task() {
    esp_hal::delay::Delay::new().delay_millis(10);
}

fn pull_scl_low() {
    let scl = cfg_select! {
        esp32p4 => esp_hal::gpio::InputSignal::I2C0_SCL,
        _ => esp_hal::gpio::InputSignal::I2CEXT0_SCL,
    };
    scl.connect_to(&esp_hal::gpio::Level::Low);
}

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::gpio::{DriveMode, Flex, OutputConfig, Pull};

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0);
        let (sda, scl) = hil_test::i2c_pins!(peripherals);

        // Test that the pin can be explicitly configured using Flex:
        let mut scl = Flex::new(scl);
        scl.apply_output_config(
            &OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::Up),
        );
        scl.set_input_enable(true);
        scl.set_output_enable(true);
        scl.set_high();

        // Create a new peripheral object with the described wiring and standard
        // I2C clock speed:
        let i2c = I2c::new(peripherals.I2C0, Config::default())
            .unwrap()
            .with_sda(sda)
            .with_scl(scl);

        Context {
            i2c,
            interrupt: peripherals.FROM_CPU_INTR2,
        }
    }

    #[test]
    fn invalid_address_returns_error(mut ctx: Context) {
        assert_eq!(
            ctx.i2c.write(0x80, &[]),
            Err(Error::AddressInvalid(I2cAddress::SevenBit(0x80)))
        );
        assert_eq!(
            ctx.i2c.read(0x80, &mut [0; 1]),
            Err(Error::AddressInvalid(I2cAddress::SevenBit(0x80)))
        );
        assert_eq!(
            ctx.i2c.write_read(0x80, &[0x77], &mut [0; 1]),
            Err(Error::AddressInvalid(I2cAddress::SevenBit(0x80)))
        );
    }

    #[test]
    fn empty_write_returns_ack_error_for_unknown_address(mut ctx: Context) {
        // on some chips we can determine the ack-check-failed reason but not on all
        // chips
        let reason = if cfg!(any(esp32, esp32s2, esp32c2, esp32c3)) {
            AcknowledgeCheckFailedReason::Unknown
        } else {
            AcknowledgeCheckFailedReason::Address
        };

        assert_eq!(
            ctx.i2c.write(NON_EXISTENT_ADDRESS, &[]),
            Err(Error::AcknowledgeCheckFailed(reason))
        );

        assert_eq!(ctx.i2c.write(DUT_ADDRESS, &[]), Ok(()));
    }

    #[test]
    fn test_read(mut ctx: Context) {
        let mut read_data = [0u8; 5];

        // have a failing read which might could leave the peripheral in an undesirable
        // state
        ctx.i2c
            .write_read(NON_EXISTENT_ADDRESS, &[0xaa], &mut read_data)
            .expect_err("Expected error for non-existent address");

        // do the real read which should succeed
        ctx.i2c
            .write_read(DUT_ADDRESS, WRITE_COMMAND, &mut read_data)
            .unwrap();

        assert_ne!(read_data, [0u8; 5]);
        assert_eq!((read_data[1], read_data[2] & 0xf0), (0x80, 0x00));
    }

    #[test]
    fn test_read_with_transactions(mut ctx: Context) {
        let mut read_data = [0u8; 5];

        // do the real read which should succeed
        ctx.i2c
            .transaction(
                DUT_ADDRESS,
                &mut [
                    Operation::Write(WRITE_COMMAND),
                    Operation::Read(&mut read_data),
                ],
            )
            .unwrap();

        assert_ne!(read_data, [0u8; 5]);
        assert_eq!((read_data[1], read_data[2] & 0xf0), (0x80, 0x00));
    }

    #[test]
    async fn async_empty_write_returns_ack_error_for_unknown_address(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        // on some chips we can determine the ack-check-failed reason but not on all
        // chips
        let reason = if cfg!(any(esp32, esp32s2, esp32c2, esp32c3)) {
            AcknowledgeCheckFailedReason::Unknown
        } else {
            AcknowledgeCheckFailedReason::Address
        };

        let should_fail = i2c.write_async(NON_EXISTENT_ADDRESS, &[]).await;
        assert_eq!(should_fail, Err(Error::AcknowledgeCheckFailed(reason)));

        assert_eq!(i2c.write_async(DUT_ADDRESS, &[]).await, Ok(()));
    }

    #[test]
    async fn async_test_read(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();
        let mut read_data = [0u8; 5];

        // have a failing read which might could leave the peripheral in an undesirable
        // state
        i2c.write_read_async(NON_EXISTENT_ADDRESS, &[0xaa], &mut read_data)
            .await
            .expect_err("Expected error for non-existent address");

        // do the real read which should succeed
        i2c.write_read_async(DUT_ADDRESS, WRITE_COMMAND, &mut read_data)
            .await
            .unwrap();

        assert_ne!(read_data, [0u8; 5]);
        assert_eq!((read_data[1], read_data[2] & 0xf0), (0x80, 0x00));
    }

    #[test]
    async fn async_test_read_with_transactions(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();
        let mut read_data = [0u8; 5];

        // do the real read which should succeed
        i2c.transaction_async(
            DUT_ADDRESS,
            &mut [
                Operation::Write(WRITE_COMMAND),
                Operation::Read(&mut read_data),
            ],
        )
        .await
        .unwrap();

        assert_ne!(read_data, [0u8; 5]);
        assert_eq!((read_data[1], read_data[2] & 0xf0), (0x80, 0x00));
    }

    #[test]
    async fn async_cancellation(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        // Read a lot of data to ensure that the transaction is cancelled mid-transfer.
        let mut read_data = [0u8; 220];

        for n_yield in 1..20 {
            defmt::info!("Running transaction to be cancelled");

            // Start a transaction that will be cancelled.
            let select_result = select(
                i2c.transaction_async(
                    DUT_ADDRESS,
                    &mut [
                        Operation::Write(WRITE_COMMAND),
                        Operation::Read(&mut read_data),
                    ],
                ),
                async {
                    // Let the transaction run for a tiny bit.
                    for _ in 0..n_yield {
                        embassy_futures::yield_now().await;
                    }
                },
            )
            .await;

            if matches!(select_result, Either::First(Ok(_))) {
                break;
            }

            // Assert that the I2C transaction was cancelled.
            hil_test::assert!(
                matches!(select_result, Either::Second(_)),
                "({}) Transaction completed with {:?}",
                n_yield,
                select_result
            );

            defmt::info!("Transaction cancelled");

            // Ping the device to ensure we don't randomly get an address failure in the next
            // iteration.
            _ = i2c.write_async(DUT_ADDRESS, &[]).await;
        }

        let mut read_data = [0u8; 5];
        // do the real read which should succeed
        i2c.transaction_async(
            DUT_ADDRESS,
            &mut [
                Operation::Write(WRITE_COMMAND),
                Operation::Read(&mut read_data),
            ],
        )
        .await
        .unwrap();

        assert_ne!(read_data, [0u8; 5]);
        assert_eq!((read_data[1], read_data[2] & 0xf0), (0x80, 0x00));
    }

    #[test]
    async fn test_timeout_when_scl_kept_low(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        i2c.apply_config(
            &Config::default()
                .with_software_timeout(SoftwareTimeout::PerByte(time::Duration::from_millis(10))),
        )
        .unwrap();

        pull_scl_low();

        let mut read_data = [0u8; 5];
        // will run into an error but it should return at least
        i2c.write_read(DUT_ADDRESS, WRITE_COMMAND, &mut read_data)
            .expect_err("Expected timeout error");
    }

    #[test]
    #[cfg(i2c_master_has_fsm_timeouts)]
    async fn test_timeout_when_scl_kept_low_with_fsm_timeout(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        i2c.apply_config(
            &Config::default()
                .with_scl_st_timeout(esp_hal::i2c::master::FsmTimeout::new_const::<16>()),
        )
        .unwrap();

        pull_scl_low();

        let mut read_data = [0u8; 5];
        // will run into an error but it should return at least
        i2c.write_read(DUT_ADDRESS, WRITE_COMMAND, &mut read_data)
            .expect_err("Expected timeout error");
    }

    #[test]
    async fn async_test_timeout_when_scl_kept_low(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        i2c.apply_config(
            &Config::default()
                .with_software_timeout(SoftwareTimeout::PerByte(time::Duration::from_millis(10))),
        )
        .unwrap();

        pull_scl_low();

        let mut read_data = [0u8; 5];
        // will run into an error but it should return at least
        i2c.write_read_async(DUT_ADDRESS, WRITE_COMMAND, &mut read_data)
            .await
            .expect_err("Expected timeout error");
    }

    #[test]
    #[timeout(10)]
    async fn no_timeout_when_preempted_for_long_time(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        let interrupt_executor =
            mk_static!(InterruptExecutor<2>, InterruptExecutor::new(ctx.interrupt));

        let spawner = interrupt_executor.start(Priority::Priority3);

        let mut ticker = Ticker::every(Duration::from_millis(10));
        for i in 0..100 {
            let mut read_data = [0u8; 5];
            let result = embassy_futures::join::join(
                i2c.write_read_async(DUT_ADDRESS, WRITE_COMMAND, &mut read_data),
                async {
                    for _ in 0..4 {
                        spawner.spawn(waiting_blocking_task().unwrap());
                        embassy_futures::yield_now().await;
                    }
                },
            )
            .await;

            assert!(
                result.0.is_ok(),
                "I2C read failed: {:?} in iteration {}",
                result.0,
                i
            );

            ticker.next().await;
        }
    }
    #[test]
    // Only APB is supported on v1, so skip the test for that version.
    #[cfg(not(i2c_master_version = "1"))]
    fn test_read_with_different_clock_sources(mut ctx: Context) {
        use esp_hal::i2c::master::ClockSource;

        // Bus frequency in kHz for each clock source.
        let configs = cfg_select! {
            any(i2c_master_version = "3", i2c_master_version = "4") => {
                [(ClockSource::Xtal, 100), (ClockSource::RcFast, 100)]
            }
            // At 100 kHz the 1 MHz REF_TICK leaves 5 ticks per half SCL cycle, which
            // puts the SDA sample point no later than the end of the SCL
            // wait-high period.
            _ => [(ClockSource::Apb, 100), (ClockSource::RefTick, 50)],
        };

        for (clock_source, khz) in configs {
            ctx.i2c
                .apply_config(
                    &Config::default()
                        .with_clock_source(clock_source)
                        .with_frequency(time::Rate::from_khz(khz)),
                )
                .unwrap_or_else(|e| panic!("{:?}: failed to apply {:?}", e, clock_source));

            let mut read_data = [0u8; 5];
            ctx.i2c
                .write_read(DUT_ADDRESS, WRITE_COMMAND, &mut read_data)
                .unwrap_or_else(|e| panic!("{:?}: failed to read with {:?}", e, clock_source));

            assert_ne!(read_data, [0u8; 5], "{:?}: read all zeros", clock_source);
            assert_eq!(
                (read_data[1], read_data[2] & 0xf0),
                (0x80, 0x00),
                "{:?}: read {:?}",
                clock_source,
                read_data
            );
        }
    }

    // Verify that `force_scl_low(true)` drives SCL low, blocking all I2C
    // clocking, and that `force_scl_low(false)` releases the line and
    // restores normal operation.
    #[test]
    fn test_force_scl_low(mut ctx: Context) {
        ctx.i2c
            .apply_config(
                &Config::default().with_software_timeout(SoftwareTimeout::PerByte(
                    time::Duration::from_millis(10),
                )),
            )
            .unwrap();

        ctx.i2c.force_scl_low(true);
        assert_eq!(
            ctx.i2c.write(DUT_ADDRESS, &[]),
            Err(Error::Timeout),
            "expected Timeout with SCL held low by pd_en"
        );

        ctx.i2c.force_scl_low(false);
        ctx.i2c.apply_config(&Config::default()).unwrap();

        let mut read_data = [0u8; 5];
        ctx.i2c
            .write_read(DUT_ADDRESS, WRITE_COMMAND, &mut read_data)
            .expect("bus should work normally after releasing SCL");
        assert_ne!(read_data, [0u8; 5]);
        assert_eq!((read_data[1], read_data[2] & 0xf0), (0x80, 0x00));
    }

    // Verify that `force_sda_low(true)` drives SDA low and that
    // `force_sda_low(false)` releases the line and restores normal operation.
    #[test]
    fn test_force_sda_low(mut ctx: Context) {
        ctx.i2c
            .apply_config(
                &Config::default().with_software_timeout(SoftwareTimeout::PerByte(
                    time::Duration::from_millis(10),
                )),
            )
            .unwrap();

        ctx.i2c.force_sda_low(true);
        let mut data = [0xFFu8; 5];
        let result = ctx.i2c.write_read(DUT_ADDRESS, WRITE_COMMAND, &mut data);
        // force_sda_low must have observable effect: either the
        // transaction failed, or it "succeeded" but with all-zero data.
        assert!(
            result.is_err() || data == [0u8; 5],
            "force_sda_low had no observable effect: got Ok with non-zero data {:?}",
            data
        );

        ctx.i2c.force_sda_low(false);
        ctx.i2c.apply_config(&Config::default()).unwrap();

        let mut read_data = [0u8; 5];
        ctx.i2c
            .write_read(DUT_ADDRESS, WRITE_COMMAND, &mut read_data)
            .expect("bus should work normally after releasing SDA");
        assert_ne!(read_data, [0u8; 5]);
        assert_eq!((read_data[1], read_data[2] & 0xf0), (0x80, 0x00));
    }
}

#[cfg(lp_i2c_master_driver_supported)]
#[embedded_test::tests(default_timeout = 3)]
mod lp_i2c {
    use esp_hal::i2c::lp_i2c::{Config, Error, LpI2c};
    #[cfg(lp_i2c_master_version = "lp_i2c")]
    use esp_hal::{i2c::lp_i2c::ConfigError, peripherals::LP_I2C0, time::Rate};
    #[cfg(lp_i2c_master_version = "rtc_i2c")]
    use esp_hal::{i2c::lp_i2c::Timing, peripherals::RTC_IO, time::Duration};

    #[cfg(esp32c6)]
    use super::NON_EXISTENT_ADDRESS;

    #[init]
    fn init() -> LpI2c<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let (sda, scl) = hil_test::i2c_pins!(peripherals);
        #[cfg(lp_i2c_master_version = "rtc_i2c")]
        let pins = {
            use esp_hal::gpio::LpPin;
            let pins =
                [sda.lp_number(), scl.lp_number()].map(|pin| RTC_IO::regs().pin(pin as usize));
            for pin in pins {
                pin.modify(|_, w| w.pad_driver().clear_bit());
            }
            pins
        };
        let config = cfg_select! {
            lp_i2c_master_version = "rtc_i2c" => Config::default()
                .with_timing(Timing::standard_mode())
                .with_timeout(Duration::from_micros(100)),
            _ => Config::default(),
        };
        let i2c = LpI2c::new(peripherals.LP_I2C0, config, sda, scl).unwrap();
        #[cfg(lp_i2c_master_version = "rtc_i2c")]
        for pin in pins {
            assert!(pin.read().pad_driver().bit());
        }
        i2c
    }

    #[cfg(lp_i2c_master_version = "lp_i2c")]
    fn config_registers() -> [u32; 5] {
        let regs = LP_I2C0::regs();
        [
            regs.scl_low_period().read().bits(),
            regs.scl_high_period().read().bits(),
            regs.sda_hold().read().bits(),
            regs.sda_sample().read().bits(),
            regs.ctr().read().bits(),
        ]
    }

    #[test]
    fn validates_addresses(mut i2c: LpI2c<'static>) {
        for address in [0x80, 0xff] {
            assert_eq!(i2c.write(address, 0x10, &[]), Err(Error::AddressInvalid));
            assert_eq!(
                i2c.write(address, 0x10, &[0xa5]),
                Err(Error::AddressInvalid)
            );
            assert_eq!(i2c.read(address, 0x10, &mut []), Err(Error::AddressInvalid));
            let mut data = [0xa5];
            assert_eq!(
                i2c.read(address, 0x10, &mut data),
                Err(Error::AddressInvalid)
            );
            assert_eq!(data, [0xa5]);
        }
        assert!(matches!(
            i2c.write(0x7f, 0x10, &[0xa5]),
            Err(Error::AckCheckFailed | Error::TimeOut)
        ));
    }

    // TODO: against the MCP4725, LP I2C transfers time out on the ESP32-S3 and are not
    // acknowledged on the ESP32-C5.
    #[test]
    #[cfg(not(any(esp32s3, esp32c5)))]
    fn read_from_mcp4725(mut i2c: LpI2c<'static>) {
        use super::{DUT_ADDRESS, WRITE_COMMAND};

        // The MCP4725 has no sub-registers, but `read` always sends a register byte before the
        // repeated start.
        let mut data = [0; 5];
        i2c.read(DUT_ADDRESS, WRITE_COMMAND[0], &mut data).unwrap();

        assert_ne!(data, [0u8; 5]);
    }

    #[test]
    #[cfg(lp_i2c_master_version = "lp_i2c")]
    fn timing_uses_selected_lp_fast_clock(mut i2c: LpI2c<'static>) {
        use esp_hal::clock::ll::{self, ClockTree, LpFastClkConfig};

        let sources = cfg_select! {
            esp32c6 => [LpFastClkConfig::RcFastClk, LpFastClkConfig::XtalD2Clk],
            _ => [LpFastClkConfig::RcFast, LpFastClkConfig::XtalD2],
        };
        let original = ClockTree::with(ll::lp_fast_clk_config).unwrap();
        let config = Config::default().with_frequency(Rate::from_khz(100));
        for source in sources {
            ClockTree::with(|clocks| ll::configure_lp_fast_clk(clocks, source));
            i2c.apply_config(&config).unwrap();
            assert_eq!(
                LP_I2C0::regs()
                    .scl_low_period()
                    .read()
                    .scl_low_period()
                    .bits() as u32,
                ll::lp_fast_clk_frequency() / 100_000 / 2 - 1,
            );
        }
        ClockTree::with(|clocks| ll::configure_lp_fast_clk(clocks, original));
        i2c.apply_config(&config).unwrap();
    }

    #[test]
    #[cfg(lp_i2c_master_version = "lp_i2c")]
    fn validates_frequency_limits(mut i2c: LpI2c<'static>) {
        let source = esp_hal::clock::ll::lp_fast_clk_frequency();
        let before = config_registers();
        for hz in [0, 1, source / 1024, source / 8 + 1, 1 << 22, u32::MAX] {
            assert_eq!(
                i2c.apply_config(&Config::default().with_frequency(Rate::from_hz(hz))),
                Err(ConfigError::FrequencyOutOfRange)
            );
            assert_eq!(config_registers(), before);
        }
        let regs = LP_I2C0::regs();
        for hz in [source / 1024 + 1, 79_999, 80_000, source / 8] {
            i2c.apply_config(&Config::default().with_frequency(Rate::from_hz(hz)))
                .unwrap();
            let half_cycle = source / hz / 2;
            let wait_high = if hz < 80_000 {
                half_cycle / 4
            } else {
                half_cycle / 2 - 2
            };
            let high = regs.scl_high_period().read();
            assert_eq!(
                regs.scl_low_period().read().scl_low_period().bits() as u32,
                half_cycle - 1
            );
            assert_eq!(high.scl_high_period().bits() as u32, half_cycle - wait_high);
            assert_eq!(high.scl_wait_high_period().bits() as u32, wait_high);
        }
    }

    #[test]
    #[cfg(esp32c6)]
    fn scl_stall_returns_timeout(mut i2c: LpI2c<'static>) {
        use esp_hal::peripherals::LP_IO;

        let scl = LP_IO::regs().gpio(7);
        for _ in 0..2 {
            let saved = scl.read().bits();
            // Disabling the input makes the controller observe SCL low.
            scl.modify(|_, w| w.fun_ie().clear_bit().mcu_ie().clear_bit());
            let result = i2c.write(NON_EXISTENT_ADDRESS, 0x10, &[0xa5]);
            scl.write(|w| unsafe { w.bits(saved) });
            assert_eq!(result, Err(Error::TimeOut));
            let interrupts = LP_I2C0::regs().int_raw().read();
            assert!(!interrupts.scl_st_to().bit());
            assert!(!interrupts.scl_main_st_to().bit());
        }
        assert_eq!(
            i2c.write(NON_EXISTENT_ADDRESS, 0x10, &[0xa5]),
            Err(Error::AckCheckFailed)
        );
        assert_eq!(
            i2c.read(NON_EXISTENT_ADDRESS, 0x10, &mut [0]),
            Err(Error::AckCheckFailed)
        );
    }
}
