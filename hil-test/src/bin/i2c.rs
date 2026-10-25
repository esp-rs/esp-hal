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

// Daniel's test device:
// const DUT_ADDRESS: u8 = 0x29;
// const READ_DATA_COMMAND: &[u8] = &[0, 0];

// HIL test device:
const DUT_ADDRESS: u8 = 0x77;
const READ_DATA_COMMAND: &[u8] = &[0xaa];

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
    fn test_read_cali(mut ctx: Context) {
        let mut read_data = [0u8; 22];

        // have a failing read which might could leave the peripheral in an undesirable
        // state
        ctx.i2c
            .write_read(NON_EXISTENT_ADDRESS, &[0xaa], &mut read_data)
            .expect_err("Expected error for non-existent address");

        // do the real read which should succeed
        ctx.i2c
            .write_read(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
            .unwrap();

        assert_ne!(read_data, [0u8; 22])
    }

    #[test]
    fn test_read_cali_with_transactions(mut ctx: Context) {
        let mut read_data = [0u8; 22];

        // do the real read which should succeed
        ctx.i2c
            .transaction(
                DUT_ADDRESS,
                &mut [
                    Operation::Write(READ_DATA_COMMAND),
                    Operation::Read(&mut read_data),
                ],
            )
            .unwrap();

        assert_ne!(read_data, [0u8; 22])
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
    async fn async_test_read_cali(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();
        let mut read_data = [0u8; 22];

        // have a failing read which might could leave the peripheral in an undesirable
        // state
        i2c.write_read_async(NON_EXISTENT_ADDRESS, &[0xaa], &mut read_data)
            .await
            .expect_err("Expected error for non-existent address");

        // do the real read which should succeed
        i2c.write_read_async(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
            .await
            .unwrap();

        assert_ne!(read_data, [0u8; 22])
    }

    #[test]
    async fn async_test_read_cali_with_transactions(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();
        let mut read_data = [0u8; 22];

        // do the real read which should succeed
        i2c.transaction_async(
            DUT_ADDRESS,
            &mut [
                Operation::Write(READ_DATA_COMMAND),
                Operation::Read(&mut read_data),
            ],
        )
        .await
        .unwrap();

        assert_ne!(read_data, [0u8; 22])
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
                        Operation::Write(READ_DATA_COMMAND),
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

            // Ping the sensor to ensure we don't randomly get an address failure in the next
            // iteration.
            _ = i2c.write_async(DUT_ADDRESS, &[]).await;
        }

        let mut read_data = [0u8; 22];
        // do the real read which should succeed
        i2c.transaction_async(
            DUT_ADDRESS,
            &mut [
                Operation::Write(READ_DATA_COMMAND),
                Operation::Read(&mut read_data),
            ],
        )
        .await
        .unwrap();

        assert_ne!(read_data, [0u8; 22])
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

        let mut read_data = [0u8; 22];
        // will run into an error but it should return at least
        i2c.write_read(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
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

        let mut read_data = [0u8; 22];
        // will run into an error but it should return at least
        i2c.write_read(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
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

        let mut read_data = [0u8; 22];
        // will run into an error but it should return at least
        i2c.write_read_async(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
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
            let mut read_data = [0u8; 22];
            let result = embassy_futures::join::join(
                i2c.write_read_async(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data),
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
    fn test_read_cali_with_different_clock_sources(mut ctx: Context) {
        use esp_hal::i2c::master::ClockSource;

        let configs = cfg_select! {
            any(i2c_master_version = "3", i2c_master_version = "4") => {
                [ClockSource::Xtal, ClockSource::RcFast]
            }
            _ => [ClockSource::Apb, ClockSource::RefTick],
        };

        for clock_source in configs {
            ctx.i2c
                .apply_config(&Config::default().with_clock_source(clock_source))
                .unwrap_or_else(|e| panic!("{:?}: failed to apply {:?}", e, clock_source));

            let mut read_data = [0u8; 22];
            ctx.i2c
                .write_read(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
                .unwrap_or_else(|e| panic!("{:?}: failed to read with {:?}", e, clock_source));

            assert_ne!(read_data, [0u8; 22]);
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

        let mut read_data = [0u8; 22];
        ctx.i2c
            .write_read(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
            .expect("bus should work normally after releasing SCL");
        assert_ne!(read_data, [0u8; 22]);
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
        let mut data = [0xFFu8; 22];
        let result = ctx
            .i2c
            .write_read(DUT_ADDRESS, READ_DATA_COMMAND, &mut data);
        // force_sda_low must have observable effect: either the
        // transaction failed, or it "succeeded" but with all-zero data.
        assert!(
            result.is_err() || data == [0u8; 22],
            "force_sda_low had no observable effect: got Ok with non-zero data {:?}",
            data
        );

        ctx.i2c.force_sda_low(false);
        ctx.i2c.apply_config(&Config::default()).unwrap();

        let mut read_data = [0u8; 22];
        ctx.i2c
            .write_read(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
            .expect("bus should work normally after releasing SDA");
        assert_ne!(read_data, [0u8; 22]);
    }

    #[test]
    #[cfg(lp_i2c_master_driver_supported)]
    fn test_read_cali_with_lp_i2c() {
        use esp_hal::i2c::lp_i2c::{Config, LpI2c};

        let peripherals = unsafe { esp_hal::peripherals::Peripherals::steal() };

        let (sda, scl) = hil_test::i2c_pins!(peripherals);

        // RTC_I2C has no usable default timing, it has to be spelled out.
        let config = cfg_select! {
            lp_i2c_master_version = "rtc_i2c" => Config::default()
                .with_timing(esp_hal::i2c::lp_i2c::Timing::standard_mode())
                .with_timeout(esp_hal::time::Duration::from_micros(100)),
            _ => Config::default(),
        };

        let mut i2c = LpI2c::new(peripherals.LP_I2C0, config, sda, scl).unwrap();

        let mut data = [0; 22];

        i2c.read(DUT_ADDRESS, READ_DATA_COMMAND[0], &mut data)
            .unwrap();

        assert_ne!(data, [0u8; 22]);
    }
}

#[cfg(i2c_slave_driver_supported)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod i2c_slave {
    use core::sync::atomic::{AtomicBool, Ordering};

    use embassy_time::Timer;
    use esp_hal::{
        delay::Delay,
        gpio::{DriveMode, Flex, Level, OutputConfig, Pull},
        i2c::slave::{Command, Config, Error, FIFO_SIZE, I2cSlave, ReadStatus},
        peripherals::{I2C0, Peripherals},
    };
    #[cfg(soc_has_pcr)]
    use esp_hal::{i2c::slave::ConfigError, peripherals::PCR};
    use hil_test::{assert, assert_eq};

    use super::*;
    const ADDRESS: u8 = 0x55;
    const RESPONSE: [u8; 33] = *b"0123456789abcdefghijklmnopqrstuvw";
    const SECOND_RESPONSE: [u8; 4] = [0x91, 0x82, 0x73, 0x64];
    const WRITE_DATA: [u8; 4] = [0xa5, 0x5a, 0xf0, 0x0f];
    const SHORT_READ_LEN: usize = 3;
    const READ_CASES: [(usize, ReadStatus); 4] = [
        (
            SHORT_READ_LEN,
            ReadStatus::LeftoverBytes((RESPONSE.len() - SHORT_READ_LEN) as u16),
        ),
        (FIFO_SIZE - 1, ReadStatus::LeftoverBytes(2)),
        (FIFO_SIZE, ReadStatus::LeftoverBytes(1)),
        (RESPONSE.len(), ReadStatus::Done),
    ];
    static MASTER_DONE: AtomicBool = AtomicBool::new(false);
    trait TestSlave {
        async fn listen(&mut self, buffer: &mut [u8]) -> Result<Command, Error>;
        async fn respond_to_read(&mut self, buffer: &[u8]) -> Result<ReadStatus, Error>;
    }

    impl TestSlave for I2cSlave<'_, Blocking> {
        async fn listen(&mut self, buffer: &mut [u8]) -> Result<Command, Error> {
            I2cSlave::listen(self, buffer)
        }

        async fn respond_to_read(&mut self, buffer: &[u8]) -> Result<ReadStatus, Error> {
            I2cSlave::respond_to_read(self, buffer)
        }
    }

    impl TestSlave for I2cSlave<'_, Async> {
        async fn listen(&mut self, buffer: &mut [u8]) -> Result<Command, Error> {
            self.listen_async(buffer).await
        }

        async fn respond_to_read(&mut self, buffer: &[u8]) -> Result<ReadStatus, Error> {
            self.respond_to_read_async(buffer).await
        }
    }

    struct Context<S = I2cSlave<'static, Blocking>> {
        slave: S,
        master: TestMaster,
        spawner: embassy_executor::SendSpawner,
    }

    impl Context {
        fn into_async(self) -> Context<I2cSlave<'static, Async>> {
            Context {
                slave: self.slave.into_async(),
                master: self.master,
                spawner: self.spawner,
            }
        }
    }

    struct TestMaster {
        sda: Flex<'static>,
        scl: Flex<'static>,
    }

    impl TestMaster {
        async fn raise_scl(&mut self) {
            self.scl.set_high();
            while self.scl.is_low() {
                Timer::after_micros(50).await;
            }
        }

        async fn clock(&mut self) -> bool {
            // Yield between edges so the blocking slave can service clock stretches.
            Timer::after_micros(50).await;
            self.raise_scl().await;
            Timer::after_micros(50).await;
            let bit = self.sda.is_high();
            self.scl.set_low();
            bit
        }

        async fn start(&mut self) {
            self.sda.set_high();
            Delay::new().delay_micros(50);
            self.raise_scl().await;
            Delay::new().delay_micros(50);
            self.sda.set_low();
            Delay::new().delay_micros(50);
            self.scl.set_low();
        }

        async fn stop(&mut self) {
            self.sda.set_low();
            Timer::after_micros(50).await;
            self.raise_scl().await;
            Timer::after_micros(50).await;
            self.sda.set_high();
            Timer::after_micros(50).await;
        }

        async fn write_byte(&mut self, byte: u8) -> bool {
            // Busy-wait so repeated START and address match can be pending together.
            for bit in (0..8).rev() {
                self.sda.set_level(Level::from(byte & (1 << bit) != 0));
                Delay::new().delay_micros(50);
                self.raise_scl().await;
                Delay::new().delay_micros(50);
                self.scl.set_low();
            }

            self.sda.set_high();
            Delay::new().delay_micros(50);
            self.raise_scl().await;
            Delay::new().delay_micros(50);
            let ack = self.sda.is_low();
            self.scl.set_low();
            ack
        }

        async fn read(&mut self, buffer: &mut [u8]) {
            self.start().await;
            assert!(self.write_byte((ADDRESS << 1) | 1).await);
            let length = buffer.len();
            for (index, byte) in buffer.iter_mut().enumerate() {
                self.sda.set_high();
                *byte = 0;
                for _ in 0..8 {
                    *byte = (*byte << 1) | u8::from(self.clock().await);
                }

                self.sda.set_level(Level::from(index + 1 == length));
                if index + 1 == length {
                    Timer::after_micros(50).await;
                    self.raise_scl().await;
                    // Leave NACK pending when a repeated START follows immediately.
                    Delay::new().delay_micros(50);
                    self.scl.set_low();
                } else {
                    self.clock().await;
                }
            }

            self.sda.set_high();
        }

        async fn write(&mut self, buffer: &[u8]) {
            Timer::after_micros(200).await;
            self.start().await;
            assert!(self.write_byte(ADDRESS << 1).await);
            for &byte in buffer {
                assert!(self.write_byte(byte).await);
            }
        }
    }

    #[cfg(soc_has_pcr)]
    fn function_clock_enabled() -> bool {
        PCR::regs()
            .i2c_sclk_conf(0)
            .read()
            .i2c_sclk_en()
            .bit_is_set()
    }

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    async fn wait_for_master() {
        while !MASTER_DONE.load(Ordering::SeqCst) {
            embassy_futures::yield_now().await;
        }
    }

    fn setup(peripherals: Peripherals) -> Context {
        MASTER_DONE.store(false, Ordering::SeqCst);
        let timg = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg.timer0);

        let (master_sda, slave_sda) = hil_test::lp_test_pins!(peripherals);
        let (master_scl, slave_scl) = hil_test::hp_test_pins!(peripherals);
        let mut sda = Flex::new(master_sda);
        let mut scl = Flex::new(master_scl);
        for pin in [&mut sda, &mut scl] {
            pin.set_high();
            pin.apply_output_config(
                &OutputConfig::default()
                    .with_drive_mode(DriveMode::OpenDrain)
                    .with_pull(Pull::Up),
            );
            pin.set_input_enable(true);
            pin.set_output_enable(true);
        }

        let slave = I2cSlave::new(peripherals.I2C0, Config::new(ADDRESS))
            .unwrap()
            .with_sda(slave_sda)
            .with_scl(slave_scl);
        // Allow scheduler delays in the software master.
        I2C0::regs()
            .to()
            .modify(|_, w| unsafe { w.time_out_value().bits(18) });

        // The master must preempt blocking slave operations.
        let executor = mk_static!(
            InterruptExecutor<2>,
            InterruptExecutor::new(peripherals.FROM_CPU_INTR2)
        );
        Context {
            slave,
            master: TestMaster { sda, scl },
            spawner: executor.start(Priority::Priority3),
        }
    }

    #[test]
    #[cfg(soc_has_pcr)]
    fn function_clock_lifetime(peripherals: Peripherals) {
        let mut i2c = peripherals.I2C0;
        // An inherited enabled gate would hide a missing clock request.
        PCR::regs()
            .i2c_sclk_conf(0)
            .modify(|_, w| w.i2c_sclk_en().clear_bit());
        assert!(!function_clock_enabled());

        assert!(matches!(
            I2cSlave::new(i2c.reborrow(), Config::new(0x80)),
            Err(ConfigError::AddressInvalid)
        ));
        assert!(!function_clock_enabled());

        for _ in 0..2 {
            let slave = I2cSlave::new(i2c.reborrow(), Config::new(ADDRESS)).unwrap();
            assert!(function_clock_enabled());
            let slave = slave.into_async();
            assert!(function_clock_enabled());
            let slave = slave.into_blocking();
            assert!(function_clock_enabled());
            drop(slave);
            assert!(!function_clock_enabled());
        }
    }

    #[embassy_executor::task]
    async fn read_response_master(mut master: TestMaster) {
        Timer::after_micros(50).await;
        for &(length, _) in &READ_CASES {
            let mut response = [0; RESPONSE.len()];
            master.read(&mut response[..length]).await;
            assert_eq!(&response[..length], &RESPONSE[..length]);
        }

        let mut second_read = [0; SECOND_RESPONSE.len()];
        master.read(&mut second_read).await;
        assert_eq!(second_read, SECOND_RESPONSE);

        // Let the slave process NACK before issuing STOP.
        Timer::after_micros(50).await;
        master.raise_scl().await;
        Timer::after_micros(200).await;
        master.scl.set_low();
        master.stop().await;

        let mut overread = [0; RESPONSE.len() + 2];
        master.read(&mut overread).await;
        assert_eq!(&overread[..RESPONSE.len()], &RESPONSE);
        assert_eq!(&overread[RESPONSE.len()..], &[0xff; 2]);

        // Let the slave process NACK before issuing repeated START.
        Timer::after_micros(200).await;

        let mut empty_read = [0; 2];
        master.read(&mut empty_read).await;
        assert_eq!(empty_read, [0xff; 2]);
        master.stop().await;
        master.write(&WRITE_DATA).await;
        master.stop().await;
        MASTER_DONE.store(true, Ordering::SeqCst);
    }

    async fn check_read_response(ctx: Context<impl TestSlave>) {
        let mut slave = ctx.slave;
        ctx.spawner.spawn(read_response_master(ctx.master).unwrap());
        let mut data = [0; WRITE_DATA.len()];

        for &(_, expected) in &READ_CASES {
            assert_eq!(slave.listen(&mut data).await, Ok(Command::Read));
            // Exercise address stretching while preparing the response.
            Delay::new().delay_micros(100);
            assert_eq!(slave.respond_to_read(&RESPONSE).await, Ok(expected));
        }

        assert_eq!(slave.listen(&mut data).await, Ok(Command::Read));
        assert_eq!(
            slave.respond_to_read(&SECOND_RESPONSE).await,
            Ok(ReadStatus::Done)
        );
        assert_eq!(slave.listen(&mut data).await, Ok(Command::Read));
        assert_eq!(
            slave.respond_to_read(&RESPONSE).await,
            Ok(ReadStatus::NeedMoreBytes)
        );
        assert_eq!(slave.listen(&mut data).await, Ok(Command::Read));
        assert_eq!(
            slave.respond_to_read(&[]).await,
            Ok(ReadStatus::NeedMoreBytes)
        );
        assert_eq!(
            slave.listen(&mut data).await,
            Ok(Command::Write(WRITE_DATA.len()))
        );
        assert_eq!(data, WRITE_DATA);
        wait_for_master().await;
    }

    #[test]
    async fn blocking_read_response(peripherals: Peripherals) {
        check_read_response(setup(peripherals)).await;
    }

    #[test]
    async fn async_read_response(peripherals: Peripherals) {
        check_read_response(setup(peripherals).into_async()).await;
    }
}
