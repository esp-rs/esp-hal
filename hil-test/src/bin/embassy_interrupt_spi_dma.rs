//! Reproduction and regression test for a sneaky issue.

//% CHIPS: esp32 esp32s2 esp32s3 esp32c3 esp32c6 esp32h2
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use embassy_time::{Duration, Instant, Timer};
use esp_hal::{
    Blocking,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    interrupt::{Priority, software::SoftwareInterruptControl},
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_rtos::embassy::InterruptExecutor;
use hil_test::mk_static;
use portable_atomic::AtomicBool;

static STOP_INTERRUPT_TASK: AtomicBool = AtomicBool::new(false);
static INTERRUPT_TASK_WORKING: AtomicBool = AtomicBool::new(false);

#[cfg(any(esp32, esp32s2, esp32s3))]
#[embassy_executor::task]
async fn interrupt_driven_task(spi: esp_hal::spi::master::SpiDma<'static, Blocking>) {
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(128);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let mut spi = spi.with_buffers(dma_rx_buf, dma_tx_buf).into_async();

    loop {
        let mut buffer: [u8; 8] = [0; 8];

        INTERRUPT_TASK_WORKING.store(true, portable_atomic::Ordering::Relaxed);
        spi.transfer_in_place_async(&mut buffer).await.unwrap();
        INTERRUPT_TASK_WORKING.store(false, portable_atomic::Ordering::Relaxed);

        if STOP_INTERRUPT_TASK.load(portable_atomic::Ordering::Relaxed) {
            break;
        }

        Timer::after(Duration::from_millis(1)).await;
    }
}

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
#[embassy_executor::task]
async fn interrupt_driven_task(i2s_tx: esp_hal::i2s::master::I2s<'static, Blocking>) {
    let (_, _, _, tx_descriptors) = dma_buffers!(128);

    let mut i2s_tx = i2s_tx.into_async().i2s_tx.build(tx_descriptors);

    loop {
        let mut buffer: [u8; 8] = [0; 8];

        INTERRUPT_TASK_WORKING.store(true, portable_atomic::Ordering::Relaxed);
        i2s_tx.write_dma_async(&mut buffer).await.unwrap();
        INTERRUPT_TASK_WORKING.store(false, portable_atomic::Ordering::Relaxed);

        if STOP_INTERRUPT_TASK.load(portable_atomic::Ordering::Relaxed) {
            break;
        }

        Timer::after(Duration::from_millis(1)).await;
    }
}

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod test {
    use super::*;

    #[test]
    async fn dma_does_not_lock_up_when_used_in_different_executors() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_int.software_interrupt0,
        );

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
                .with_frequency(Rate::from_khz(10000))
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
                .with_frequency(Rate::from_khz(10000))
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_dma(dma_channel2);

        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        let other_peripheral = esp_hal::i2s::master::I2s::new(
            peripherals.I2S0,
            dma_channel2,
            esp_hal::i2s::master::Config::new_tdm_philips()
                .with_sample_rate(Rate::from_khz(8))
                .with_data_format(esp_hal::i2s::master::DataFormat::Data16Channel16)
                .with_channels(esp_hal::i2s::master::Channels::STEREO),
        )
        .unwrap();

        let interrupt_executor = mk_static!(
            InterruptExecutor<1>,
            InterruptExecutor::new(sw_int.software_interrupt1)
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
                break;
            }

            i = i.wrapping_add(1);
        }

        // make sure the other peripheral didn't get stuck
        STOP_INTERRUPT_TASK.store(true, portable_atomic::Ordering::Relaxed);
        while INTERRUPT_TASK_WORKING.load(portable_atomic::Ordering::Relaxed) {}
    }

    // Reproducer of https://github.com/esp-rs/esp-hal/issues/2369
    #[cfg(multi_core)]
    #[test]
    async fn dma_does_not_lock_up_on_core_1() {
        use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
        use esp_hal::{
            peripherals::{CPU_CTRL, SPI2},
            system::{Cpu, CpuControl, Stack},
        };

        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                type DmaChannel<'a> = esp_hal::peripherals::DMA_SPI2<'a>;
            } else {
                type DmaChannel<'a> = esp_hal::peripherals::DMA_CH0<'a>;
            }
        }

        const BUFFER_SIZE: usize = 256;

        pub struct SpiPeripherals {
            pub spi: SPI2<'static>,
            pub dma_channel: DmaChannel<'static>,
        }

        #[embassy_executor::task]
        async fn run_spi(
            peripherals: SpiPeripherals,
            finished: &'static Signal<CriticalSectionRawMutex, ()>,
        ) {
            let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(3200);
            let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
            let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

            let mut spi = Spi::new(
                peripherals.spi,
                Config::default()
                    .with_frequency(Rate::from_khz(100))
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
                finished.signal(());
            }
        }

        let peripherals = esp_hal::init(esp_hal::Config::default());

        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_int.software_interrupt0,
        );

        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                let dma_channel = peripherals.DMA_SPI2;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let transfer_finished = &*mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let spi_peripherals = SpiPeripherals {
            spi: peripherals.SPI2,
            dma_channel,
        };

        let app_core_stack = mk_static!(Stack<8192>, Stack::new());

        esp_rtos::start_second_core(
            peripherals.CPU_CTRL,
            #[cfg(xtensa)]
            sw_int.software_interrupt0,
            sw_int.software_interrupt1,
            app_core_stack,
            || {
                use esp_hal::interrupt::Priority;
                let software_interrupt = sw_int.software_interrupt2;
                let hp_executor = mk_static!(
                    InterruptExecutor<2>,
                    InterruptExecutor::new(software_interrupt)
                );
                let high_pri_spawner = hp_executor.start(Priority::Priority2);

                // spi runs as high priority task
                high_pri_spawner
                    .spawn(run_spi(spi_peripherals, transfer_finished))
                    .ok();
            },
        );

        // Wait for a few SPI transfers to happen
        for _ in 0..5 {
            transfer_finished.wait().await;
        }

        // make sure the other peripheral didn't get stuck
        STOP_INTERRUPT_TASK.store(true, portable_atomic::Ordering::Relaxed);
        while INTERRUPT_TASK_WORKING.load(portable_atomic::Ordering::Relaxed) {}

        unsafe {
            // Park the second core, we don't need it anymore
            CpuControl::new(CPU_CTRL::steal()).park_core(Cpu::AppCpu);
        }
    }
}
