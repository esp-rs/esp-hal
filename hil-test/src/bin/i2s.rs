//! I2S Loopback and I2S parallel interface tests
//!
//! This test uses I2S TX to transmit known data to I2S RX (forced to slave mode
//! with loopback mode enabled).

//% CHIP_FILTER: i2s_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::{
        Async,
        delay::Delay,
        dma::{DmaDescriptor, DmaRxBuf, DmaTxBuf, DmaTxStreamBuf},
        dma_rx_stream_buffer,
        dma_tx_stream_buffer,
        gpio::{AnyPin, NoPin, Pin},
        i2s::master::{Channels, Config, DataFormat, I2s, I2sTx},
        peripherals::I2S0,
        time::Rate,
    };

    cfg_select! {
        any(esp32, esp32s2) => {
            type DmaChannel0<'d> = esp_hal::peripherals::DMA_I2S0<'d>;
        }
        _ => {
            type DmaChannel0<'d> = esp_hal::peripherals::DMA_CH0<'d>;
        }
    }

    const BUFFER_SIZE: usize = 2000;

    #[derive(Clone)]
    struct SampleSource {
        i: u8,
    }

    impl SampleSource {
        // choose values which DON'T restart on every descriptor buffer's start
        const ADD: u8 = 5;
        const CUT_OFF: u8 = 113;

        fn new() -> Self {
            Self { i: 1 }
        }
    }

    impl Iterator for SampleSource {
        type Item = u8;

        fn next(&mut self) -> Option<Self::Item> {
            let i = self.i;
            self.i = (i + Self::ADD) % Self::CUT_OFF;
            Some(i)
        }
    }

    #[embassy_executor::task]
    async fn writer(mut tx_buffer: DmaTxStreamBuf, i2s_tx: I2sTx<'static, Async>) {
        let mut samples = SampleSource::new();
        tx_buffer.push_with(|buf| {
            for b in buf.iter_mut() {
                *b = samples.next().unwrap();
            }
            buf.len()
        });

        let mut tx_transfer = i2s_tx.write(tx_buffer).ok().unwrap();

        loop {
            let before = tx_transfer.available_bytes();
            tx_transfer.wait_for_available_async().await.ok().unwrap();
            let after1 = tx_transfer.available_bytes();
            tx_transfer.wait_for_available_async().await.ok().unwrap();
            let after2 = tx_transfer.available_bytes();
            assert!(after1 >= before);
            assert!(after2 >= after1);

            if tx_transfer.available_bytes() > 0 {
                tx_transfer.push_with(|buffer| {
                    for b in buffer.iter_mut() {
                        *b = samples.next().unwrap();
                    }
                    buffer.len()
                });
            }
        }
    }

    struct Context {
        dout: AnyPin<'static>,
        dma_channel: DmaChannel0<'static>,
        i2s: I2S0<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );

        let dma_channel = cfg_select! {
            i2s_dma_engine = "I2S_DMA" => {
                peripherals.DMA_I2S0
            },
            _ => {
                peripherals.DMA_CH0
            },
        };

        #[cfg(not(esp32c5))]
        let (_, dout) = hil_test::common_test_pins!(peripherals);

        // there are some random pulses counted on ESP32-C5 in our HIL setup
        // with the common test pin, so just use a fixed GPIO pin which is not used for anything
        // else in this test on that chip in the hope it will fix it
        #[cfg(esp32c5)]
        let dout = peripherals.GPIO6;

        Context {
            dout: dout.degrade(),
            dma_channel,
            i2s: peripherals.I2S0,
        }
    }

    #[test]
    async fn test_i2s_loopback_async(ctx: Context) {
        let spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };

        let rx_buffer = dma_rx_stream_buffer!(BUFFER_SIZE, 128);
        let tx_buffer = dma_tx_stream_buffer!(BUFFER_SIZE, 128);

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_signal_loopback(true)
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap()
        .into_async();

        let (din, dout) = unsafe { ctx.dout.split() };

        let i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(dout)
            .build();

        let i2s_rx = i2s
            .i2s_rx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_din(din)
            .build();

        let mut rx_transfer = i2s_rx.read(rx_buffer).ok().unwrap();
        spawner.spawn(writer(tx_buffer, i2s_tx).unwrap());

        let mut rcv = [0u8; BUFFER_SIZE];
        let mut sample_idx = 0;
        let mut samples = SampleSource::new();
        for _i in 0..30 {
            rx_transfer.wait_for_available_async().await.unwrap();
            let avl = rx_transfer.available_bytes();
            let len = rx_transfer.pop(&mut rcv[..avl]);

            // WORKAROUND - in loopback mode (only), the ESP32's I2S RX seems to randomly miss the
            // first sample. Not ideal but skip those extra zeroes to have this tested to the extend
            // possible.
            #[cfg(esp32)]
            let (rcv, len) = if _i == 0 && rcv[0] == 0 {
                let zeros = rcv.iter().take_while(|&&b| b == 0).count();
                (&rcv[zeros..], len - zeros)
            } else {
                (&rcv[..], len)
            };

            for &b in &rcv[..len] {
                let expected = samples.next().unwrap();
                assert_eq!(
                    b, expected,
                    "Sample #{} does not match ({} != {})",
                    sample_idx, b, expected
                );
                sample_idx += 1;
            }
        }
    }

    #[test]
    fn test_i2s_loopback(ctx: Context) {
        let rx_buffer = dma_rx_stream_buffer!(BUFFER_SIZE * 4, 1000);
        let mut tx_buffer = dma_tx_stream_buffer!(BUFFER_SIZE * 4, 1000);

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_signal_loopback(true)
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap();

        let (din, dout) = unsafe { ctx.dout.split() };

        let i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(dout)
            .build();

        let i2s_rx = i2s
            .i2s_rx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_din(din)
            .build();

        let mut samples = SampleSource::new();
        tx_buffer.push_with(|buf| {
            for b in buf.iter_mut() {
                *b = samples.next().unwrap();
            }
            buf.len()
        });

        let mut rcv = [0u8; 11000];
        let mut filler = [0x1u8; 12000];

        let mut rx_transfer = i2s_rx.read(rx_buffer).ok().unwrap();

        // no data available yet
        assert_eq!(0, rx_transfer.available_bytes());

        let mut tx_transfer = i2s_tx.write(tx_buffer).ok().unwrap();

        let mut iteration = 0;
        let mut sample_idx = 0;
        let mut check_samples = SampleSource::new();

        loop {
            loop {
                let tx_avail = tx_transfer.available_bytes();
                if tx_avail > 0 {
                    for b in &mut filler[0..tx_avail].iter_mut() {
                        *b = samples.next().unwrap();
                    }
                    tx_transfer.push(&filler[0..tx_avail]);
                } else {
                    break;
                }
            }

            // test calling available multiple times doesn't break anything
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            let rx_avail = rx_transfer.available_bytes();

            // make sure there are more than one descriptor buffers ready to pop
            if rx_avail > 0 {
                rcv.fill(0xff);
                let len = rx_transfer.pop(&mut rcv[..rx_avail]);
                assert!(len > 0);

                // WORKAROUND - in loopback mode (only), the ESP32's I2S RX seems to randomly miss
                // the first sample, causing this test to fail spuriously. Not ideal
                // but skip those zeroes to have this tested to the extend possible.
                #[cfg(esp32)]
                let (rcv, len) = if iteration == 0 && rcv[0] == 0 {
                    let zeros = rcv.iter().take_while(|&&b| b == 0).count();
                    (&rcv[zeros..], len - zeros)
                } else {
                    (&rcv[..], len)
                };

                for &b in &rcv[..len] {
                    let expected = check_samples.next().unwrap();
                    assert_eq!(
                        b, expected,
                        "Sample #{} does not match ({} != {})",
                        sample_idx, b, expected
                    );
                    sample_idx += 1;
                }

                iteration += 1;

                if iteration == 1 {
                    // delay to make it likely `available` will need to handle more than one
                    // descriptor next time
                    Delay::new().delay_millis(60);
                }
            }

            if iteration > 30 {
                break;
            }
        }
    }

    #[test]
    #[cfg(not(any(esp32, esp32s2)))]
    fn test_i2s_rx_half_sample_bits_regression(ctx: Context) {
        // Regression test for rx_half_sample_bits configuration bug.
        // Validates that TX and RX half_sample_bits registers are configured identically.
        let _i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap();

        // Access registers directly to read back the configured values
        let regs = esp_hal::peripherals::I2S0::regs();
        let tx_conf1 = regs.tx_conf1().read();
        let rx_conf1 = regs.rx_conf1().read();

        let tx_half_sample_bits = tx_conf1.tx_half_sample_bits().bits();
        let rx_half_sample_bits = rx_conf1.rx_half_sample_bits().bits();

        // These MUST be identical - if they differ, the rx_half_sample_bits bug is present
        assert_eq!(
            tx_half_sample_bits, rx_half_sample_bits,
            "16-bit Stereo: half_sample_bits mismatch TX={}, RX={} (should be identical for proper timing)",
            tx_half_sample_bits, rx_half_sample_bits
        );

        // For Data16Channel16 + STEREO: (16 * 2) / 2 - 1 = 15
        let expected_value = 15u8;
        assert_eq!(
            tx_half_sample_bits, expected_value,
            "16-bit Stereo: TX half_sample_bits={}, expected {}",
            tx_half_sample_bits, expected_value
        );
        assert_eq!(
            rx_half_sample_bits, expected_value,
            "16-bit Stereo: RX half_sample_bits={}, expected {}",
            rx_half_sample_bits, expected_value
        );
    }

    #[test]
    fn test_i2s_read_one_shot_multiple(ctx: Context) {
        let buffer = hil_test::mk_static!([u8; 8000], [0u8; 8000]);
        let descr = hil_test::mk_static!([DmaDescriptor; 4], [DmaDescriptor::EMPTY; 4]);
        let mut rx_buffer = DmaRxBuf::new(descr, buffer).unwrap();

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap();

        // reading WS as input will generate a certain bit pattern on the data line which we can
        // check was received correctly
        let (din, ws) = unsafe { ctx.dout.split() };

        let mut i2s_rx = i2s
            .i2s_rx
            .with_bclk(NoPin)
            .with_ws(ws)
            .with_din(din)
            .build();

        for _ in 0..10 {
            let rx_transfer = i2s_rx.read(rx_buffer).unwrap();
            let (res, i2s_rx_back, done_rx) = rx_transfer.wait();
            assert!(res.is_ok(), "I2S read transfer failed: {:?}", res.err());

            assert!(done_rx.number_of_received_bytes() != 0);

            let mut tmp = [0u8; 8000];
            let read = done_rx.read_received_data(&mut tmp);

            // I2S RX might not fill each buffer for every descriptor - so the read byte count might
            // be less than the buffer size, but it should never be zero if the transfer
            // completed successfully.
            assert!(read != 0);

            let l = read - read % 4;
            for chunk in tmp[..l].chunks(4) {
                assert_eq!(chunk, [1, 0, 254, 255]);
            }

            i2s_rx = i2s_rx_back;
            rx_buffer = done_rx;
        }
    }

    #[test]
    async fn test_i2s_read_one_shot_multiple_async(ctx: Context) {
        let buffer = hil_test::mk_static!([u8; 8000], [0u8; 8000]);
        let descr = hil_test::mk_static!([DmaDescriptor; 4], [DmaDescriptor::EMPTY; 4]);
        let mut rx_buffer = DmaRxBuf::new(descr, buffer).unwrap();

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap()
        .into_async();

        // reading WS as input will generate a certain bit pattern on the data line which we can
        // check was received correctly
        let (din, ws) = unsafe { ctx.dout.split() };

        let mut i2s_rx = i2s
            .i2s_rx
            .with_bclk(NoPin)
            .with_ws(ws)
            .with_din(din)
            .build();

        for _ in 0..10 {
            let mut rx_transfer = i2s_rx.read(rx_buffer).unwrap();
            rx_transfer.wait_for_done_async().await.unwrap();
            assert_eq!(rx_transfer.is_done(), true);
            let (res, i2s_rx_back, done_rx) = rx_transfer.wait_async().await;
            assert!(res.is_ok(), "I2S read transfer failed: {:?}", res.err());

            assert!(done_rx.number_of_received_bytes() != 0);

            let mut tmp = [0u8; 8000];
            let read = done_rx.read_received_data(&mut tmp);

            // I2S RX might not fill each buffer for every descriptor - so the read byte count might
            // be less than the buffer size, but it should never be zero if the transfer
            // completed successfully.
            assert!(read != 0);

            let l = read - read % 4;
            for chunk in tmp[..l].chunks(4) {
                assert_eq!(chunk, [1, 0, 254, 255]);
            }

            i2s_rx = i2s_rx_back;
            rx_buffer = done_rx;
        }
    }

    // On chips supporting PCNT we check the number of written bytes, otherwise just make sure the
    // write completes.
    #[test]
    fn test_i2s_write_one_shot(ctx: Context) {
        let buffer = hil_test::mk_static!([u8; 8000], [1u8; 8000]);
        let descr = hil_test::mk_static!([DmaDescriptor; 4], [DmaDescriptor::EMPTY; 4]);
        let tx_buffer = DmaTxBuf::new(descr, buffer).unwrap();

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap();

        let (other, dout) = unsafe { ctx.dout.split() };
        let mut counter = super::EdgeCounter::new(other);

        let i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(dout)
            .build();

        counter.clear();
        counter.check(0);

        let tx_transfer = i2s_tx.write(tx_buffer).unwrap();
        esp_hal::delay::Delay::new().delay_millis(500);
        let (res, _, _done_tx) = tx_transfer.wait();
        assert!(res.is_ok(), "I2S write transfer failed: {:?}", res.err());
        counter.check(8000);
    }

    // On chips supporting PCNT we check the number of written bytes, otherwise just make sure the
    // write completes.
    #[test]
    async fn test_i2s_write_one_shot_async(ctx: Context) {
        let buffer = hil_test::mk_static!([u8; 8000], [1u8; 8000]);
        let descr = hil_test::mk_static!([DmaDescriptor; 4], [DmaDescriptor::EMPTY; 4]);
        let tx_buffer = DmaTxBuf::new(descr, buffer).unwrap();

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap()
        .into_async();

        let (other, dout) = unsafe { ctx.dout.split() };
        let mut counter = super::EdgeCounter::new(other);

        let i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(dout)
            .build();

        counter.clear();
        counter.check(0);

        let mut tx_transfer = i2s_tx.write(tx_buffer).unwrap();
        tx_transfer.wait_for_done_async().await.unwrap();
        assert_eq!(tx_transfer.is_done(), true);
        let (res, _, _done_tx) = tx_transfer.wait_async().await;
        assert!(res.is_ok(), "I2S write transfer failed: {:?}", res.err());
        counter.check(8000);
    }

    // On chips supporting PCNT we check the number of written bytes, otherwise just make sure the
    // write completes.
    #[test]
    async fn test_i2s_write_one_shot_multiple(ctx: Context) {
        let buffer = hil_test::mk_static!([u8; 4], [1u8; 4]);
        let descr = hil_test::mk_static!([DmaDescriptor; 4], [DmaDescriptor::EMPTY; 4]);
        let mut tx_buffer = DmaTxBuf::new(descr, buffer).unwrap();

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap();

        let (other, dout) = unsafe { ctx.dout.split() };
        let mut counter = super::EdgeCounter::new(other);

        let mut i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(dout)
            .build();

        counter.clear();
        counter.check(0);

        let mut compare = 4;
        for _ in 0..10 {
            let tx_transfer = i2s_tx.write(tx_buffer).unwrap();
            let (res, i2s_tx_back, tx_buffer_back) = tx_transfer.wait();
            assert!(res.is_ok(), "I2S write transfer failed: {:?}", res.err());
            counter.check(compare);
            compare += 4;

            tx_buffer = tx_buffer_back;
            i2s_tx = i2s_tx_back;
        }
    }

    // On chips supporting PCNT we check the number of written bytes, otherwise just make sure the
    // write completes.
    #[test]
    async fn test_i2s_write_one_shot_multiple_async(ctx: Context) {
        let buffer = hil_test::mk_static!([u8; 4], [1u8; 4]);
        let descr = hil_test::mk_static!([DmaDescriptor; 4], [DmaDescriptor::EMPTY; 4]);
        let mut tx_buffer = DmaTxBuf::new(descr, buffer).unwrap();

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap()
        .into_async();

        let (other, dout) = unsafe { ctx.dout.split() };
        let mut counter = super::EdgeCounter::new(other);

        let mut i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(dout)
            .build();

        counter.clear();
        counter.check(0);

        let mut compare = 4;
        for _ in 0..10 {
            let mut tx_transfer = i2s_tx.write(tx_buffer).unwrap();
            tx_transfer.wait_for_done_async().await.unwrap();
            assert_eq!(tx_transfer.is_done(), true);
            let (res, i2s_tx_back, tx_buffer_back) = tx_transfer.wait_async().await;
            assert!(res.is_ok(), "I2S write transfer failed: {:?}", res.err());
            counter.check(compare);
            compare += 4;
            tx_buffer = tx_buffer_back;
            i2s_tx = i2s_tx_back;
        }
    }
}

#[cfg(esp32)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod parallel_tests {
    use esp_hal::{
        gpio::NoPin,
        i2s::parallel::{I2sParallel, TxSixteenBits},
        peripherals::{DMA_I2S0, I2S0},
        time::Rate,
    };
    struct Context {
        dma_channel: DMA_I2S0<'static>,
        i2s: I2S0<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );

        let dma_channel = peripherals.DMA_I2S0;

        Context {
            dma_channel,
            i2s: peripherals.I2S0,
        }
    }

    #[test]
    async fn driver_does_not_hang_when_async(ctx: Context) {
        let pins = TxSixteenBits::new(
            NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin,
            NoPin, NoPin, NoPin, NoPin,
        );
        let i2s = I2sParallel::new(ctx.i2s, ctx.dma_channel, Rate::from_mhz(20), pins, NoPin)
            .into_async();

        // Try sending an empty buffer, as an edge case
        let tx_buf = esp_hal::dma_tx_buffer!(4096).unwrap();
        let mut xfer = i2s
            .send(tx_buf)
            .map_err(|_| "failed to send empty buffer")
            .unwrap();
        xfer.wait_for_done().await.unwrap();

        let (i2s, mut tx_buf) = xfer.wait();

        // Now send some data
        tx_buf.fill(&[0x12; 128]);

        let mut xfer = i2s
            .send(tx_buf)
            .map_err(|_| "failed to send buffer")
            .unwrap();
        xfer.wait_for_done().await.unwrap();
    }
}

struct EdgeCounter<'a> {
    #[cfg(pcnt_driver_supported)]
    unit: esp_hal::pcnt::unit::Unit<'a, 0>,

    phantom: core::marker::PhantomData<&'a ()>,
}

#[cfg(pcnt_driver_supported)]
impl<'a> EdgeCounter<'a> {
    fn new<'i>(input: impl esp_hal::gpio::interconnect::PeripheralInput<'i>) -> Self {
        let pcnt = esp_hal::pcnt::Pcnt::new(unsafe { esp_hal::peripherals::PCNT::steal() });
        let unit = pcnt.unit0;
        unit.channel0.set_edge_signal(input);
        unit.channel0.set_input_mode(
            esp_hal::pcnt::channel::EdgeMode::Hold,
            esp_hal::pcnt::channel::EdgeMode::Increment,
        );
        unit.clear();

        Self {
            unit,
            phantom: Default::default(),
        }
    }

    fn clear(&mut self) {
        self.unit.clear();
    }

    fn count(&self) -> i32 {
        self.unit.counter.get() as i32
    }

    fn check(&self, expected: i32) {
        let count = self.count();
        assert_eq!(count, expected, "Edge count does not match expected value");
    }
}

#[cfg(not(pcnt_driver_supported))]
impl<'a> EdgeCounter<'a> {
    fn new<'i>(_input: impl esp_hal::gpio::interconnect::PeripheralInput<'i>) -> Self {
        Self {
            phantom: Default::default(),
        }
    }

    fn clear(&mut self) {
        // NOOP
    }

    fn check(&self, _expected: i32) {}
}
