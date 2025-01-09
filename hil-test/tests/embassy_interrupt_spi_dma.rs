//! Reproduction and regression test for a sneaky issue.

//% CHIPS: esp32 esp32s2 esp32s3 esp32c3 esp32c6 esp32h2
//% FEATURES(integrated): unstable embassy integrated-timers
//% FEATURES(generic): unstable embassy generic-queue

#![no_std]
#![no_main]

use embassy_time::{Duration, Instant, Ticker};
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    interrupt::{software::SoftwareInterruptControl, Priority},
    peripheral::Peripheral,
    spi::{
        master::{Config, Spi},
        Mode,
    },
    time::RateExtU32,
    timer::AnyTimer,
    Async,
};
use esp_hal_embassy::InterruptExecutor;
use hil_test as _;
use portable_atomic::AtomicBool;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

static INTERRUPT_TASK_WORKING: AtomicBool = AtomicBool::new(false);

#[cfg(any(esp32, esp32s2, esp32s3))]
#[embassy_executor::task]
async fn interrupt_driven_task(spi: esp_hal::spi::master::SpiDma<'static, Async>) {
    let mut ticker = Ticker::every(Duration::from_millis(1));

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(128);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let mut spi = spi.with_buffers(dma_rx_buf, dma_tx_buf);

    loop {
        let mut buffer: [u8; 8] = [0; 8];

        INTERRUPT_TASK_WORKING.fetch_not(portable_atomic::Ordering::Release);
        spi.transfer_in_place_async(&mut buffer).await.unwrap();
        INTERRUPT_TASK_WORKING.fetch_not(portable_atomic::Ordering::Acquire);

        ticker.next().await;
    }
}

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
#[embassy_executor::task]
async fn interrupt_driven_task(mut i2s_tx: esp_hal::i2s::master::I2sTx<'static, Async>) {
    let mut ticker = Ticker::every(Duration::from_millis(1));

    loop {
        let mut buffer: [u8; 8] = [0; 8];

        INTERRUPT_TASK_WORKING.fetch_not(portable_atomic::Ordering::Release);
        i2s_tx.write_dma_async(&mut buffer).await.unwrap();
        INTERRUPT_TASK_WORKING.fetch_not(portable_atomic::Ordering::Acquire);

        ticker.next().await;
    }
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod test {
    use super::*;

    #[test]
    async fn dma_does_not_lock_up_when_used_in_different_executors() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(systimer)] {
                use esp_hal::timer::systimer::SystemTimer;
                let systimer = SystemTimer::new(peripherals.SYSTIMER);
                esp_hal_embassy::init([
                    AnyTimer::from(systimer.alarm0),
                    AnyTimer::from(systimer.alarm1),
                ]);
            } else {
                use esp_hal::timer::timg::TimerGroup;
                let timg0 = TimerGroup::new(peripherals.TIMG0);
                esp_hal_embassy::init([
                    AnyTimer::from(timg0.timer0),
                    AnyTimer::from(timg0.timer1),
                ]);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel1 = peripherals.DMA_SPI2;
                let dma_channel2 = peripherals.DMA_SPI3;
            } else {
                let dma_channel1 = peripherals.DMA_CH0;
                let dma_channel2 = peripherals.DMA_CH1;
            }
        }

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(1024);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let (_, mosi) = hil_test::common_test_pins!(peripherals);

        let mut spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(10000.kHz())
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_miso(unsafe { mosi.clone_unchecked() })
        .with_mosi(mosi)
        .with_dma(dma_channel1)
        .with_buffers(dma_rx_buf, dma_tx_buf)
        .into_async();

        #[cfg(any(esp32, esp32s2, esp32s3))]
        let other_peripheral = Spi::new(
            peripherals.SPI3,
            Config::default()
                .with_frequency(10000.kHz())
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_dma(dma_channel2)
        .into_async();

        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        let other_peripheral = {
            let (_, rx_descriptors, _, tx_descriptors) = dma_buffers!(128);

            let other_peripheral = esp_hal::i2s::master::I2s::new(
                peripherals.I2S0,
                esp_hal::i2s::master::Standard::Philips,
                esp_hal::i2s::master::DataFormat::Data8Channel8,
                8u32.kHz(),
                dma_channel2,
                rx_descriptors,
                tx_descriptors,
            )
            .into_async()
            .i2s_tx
            .build();

            other_peripheral
        };

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let interrupt_executor = mk_static!(
            InterruptExecutor<1>,
            InterruptExecutor::new(sw_ints.software_interrupt1)
        );

        let spawner = interrupt_executor.start(Priority::Priority3);

        spawner
            .spawn(interrupt_driven_task(other_peripheral))
            .unwrap();

        let start = Instant::now();
        let mut buffer: [u8; 1024] = [0; 1024];
        let mut dst_buffer: [u8; 1024] = [0; 1024];
        let mut i = 0;
        loop {
            buffer.fill(i);
            dst_buffer.fill(i.wrapping_add(1));
            spi.transfer_async(&mut dst_buffer, &buffer).await.unwrap();
            // make sure the transfer didn't end prematurely
            assert!(dst_buffer.iter().all(|&v| v == i));

            if start.elapsed() > Duration::from_secs(1) {
                // make sure the other peripheral didn't get stuck
                while INTERRUPT_TASK_WORKING.load(portable_atomic::Ordering::Acquire) {}
                break;
            }

            i = i.wrapping_add(1);
        }
    }

    // Reproducer of https://github.com/esp-rs/esp-hal/issues/2369
    #[cfg(multi_core)]
    #[test]
    async fn dma_does_not_lock_up_on_core_1() {
        use embassy_time::Timer;
        use esp_hal::peripherals::SPI2;
        use portable_atomic::{AtomicU32, Ordering};

        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                use esp_hal::dma::Spi2DmaChannel as DmaChannel;
            } else {
                type DmaChannel = esp_hal::dma::DmaChannel0;
            }
        }

        const BUFFER_SIZE: usize = 256;
        static LOOP_COUNT: AtomicU32 = AtomicU32::new(0);

        pub struct SpiPeripherals {
            pub spi: SPI2,
            pub dma_channel: DmaChannel,
        }

        #[embassy_executor::task]
        async fn run_spi(peripherals: SpiPeripherals) {
            let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(3200);
            let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
            let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

            let mut spi = Spi::new(
                peripherals.spi,
                Config::default()
                    .with_frequency(100.kHz())
                    .with_mode(Mode::_0),
            )
            .unwrap()
            .with_dma(peripherals.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf)
            .into_async();

            let send_buffer = mk_static!([u8; BUFFER_SIZE], [0u8; BUFFER_SIZE]);
            loop {
                let mut buffer = [0; 8];
                embedded_hal_async::spi::SpiBus::transfer(&mut spi, &mut buffer, send_buffer)
                    .await
                    .unwrap();
                LOOP_COUNT.fetch_add(1, Ordering::Relaxed);
            }
        }

        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(systimer)] {
                use esp_hal::timer::systimer::SystemTimer;
                let systimer = SystemTimer::new(peripherals.SYSTIMER);
                esp_hal_embassy::init([
                    AnyTimer::from(systimer.alarm0),
                    AnyTimer::from(systimer.alarm1),
                ]);
            } else {
                use esp_hal::timer::timg::TimerGroup;
                let timg0 = TimerGroup::new(peripherals.TIMG0);
                esp_hal_embassy::init([
                    AnyTimer::from(timg0.timer0),
                    AnyTimer::from(timg0.timer1),
                ]);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                let dma_channel = peripherals.DMA_SPI2;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let spi_peripherals = SpiPeripherals {
            spi: peripherals.SPI2,
            dma_channel,
        };

        let cpu1_fnctn = {
            move || {
                use esp_hal::interrupt::Priority;
                use esp_hal_embassy::InterruptExecutor;
                let sw_ints = esp_hal::interrupt::software::SoftwareInterruptControl::new(
                    peripherals.SW_INTERRUPT,
                );
                let software_interrupt = sw_ints.software_interrupt2;
                let hp_executor = mk_static!(
                    InterruptExecutor<2>,
                    InterruptExecutor::new(software_interrupt)
                );
                let high_pri_spawner = hp_executor.start(Priority::Priority2);

                // hub75 runs as high priority task
                high_pri_spawner.spawn(run_spi(spi_peripherals)).ok();

                // This loop is necessary to avoid parking the core after creating the interrupt
                // executor.
                loop {}
            }
        };

        use esp_hal::cpu_control::{CpuControl, Stack};
        const DISPLAY_STACK_SIZE: usize = 8192;
        let app_core_stack = mk_static!(Stack<DISPLAY_STACK_SIZE>, Stack::new());
        let cpu_control = CpuControl::new(peripherals.CPU_CTRL);
        let mut _cpu_control = cpu_control;

        #[allow(static_mut_refs)]
        let _guard = _cpu_control
            .start_app_core(app_core_stack, cpu1_fnctn)
            .unwrap();

        let mut last = 0u32;
        for _ in 0..5 {
            Timer::after(Duration::from_millis(200)).await;

            let next = LOOP_COUNT.load(Ordering::Relaxed);
            assert_ne!(next, last, "stuck");
            last = next;
        }
    }
}
