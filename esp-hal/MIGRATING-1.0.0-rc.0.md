# Migration Guide from 1.0.0-rc.0 to {{currentVersion}}

## RNG changes

Random number generator objects can now be created anywhere. The `RNG` peripheral singleton
is only necessary to enable the cryptographically secure random number generator.

 - `Rng` can be constructed without any constraints.
 - `Trng` can only be constructed when it can be ensured to generate true random numbers.

A new `TrngSource` object has been added. Users can use `TrngSource::new` to create the object,
and as long as it is alive, `Trng::try_new` will return `Ok` and will provide a true random number
generator interface.

The previous way to obtain RNG object has changed like so:

```diff
-let mut rng = Rng::new(peripherals.RNG);
+let rng = Rng::new();

-let mut trng = Trng::new(peripherals.RNG, peripherals.ADC1);
+// Once:
+let trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1);
+// As long as `trng_source` is alive:
+let trng = Tnrg::try_new().unrwap();
```

## AES changes

The `esp_hal::aes::Aes` and `esp_hal::aes::AesDma` drivers has been slightly reworked:

- `Mode` has been replaced by `Operation`. Operation has `Encrypt` and `Decrypt` variants, but the key length is no longer part of the enum. The key length is specified by the key. AesDma now takes this `Operation`.
- `Aes::process` has been split into `encrypt` and `decrypt`. These functions no longer take a mode parameter.
- `AesDma::write_block` and `AesDma::write_key` have been removed.
- `AesDma::process` now takes `DmaCipherState` which includes information to initialize the block cipher mode of operation.

```diff
-aes.process(block, Mode::Encrypt128, key);
+aes.encrypt(block, key_16_bytes);

-aes.process(block, Mode::Decrypt256, key);
+aes.decrypt(block, key_32_bytes);
```

```diff
+use esp_hal::aes::dma::DmaCipherState;
+use esp_hal::aes::cipher_modes::Ecb;

 let transfer = aes_dma
     .process(
         1,
         output,
         input,
         Operation::Encrypt,
-        CipherMode::Ecb,
+        &DmaCipherState::from(Ecb),
         key,
     )
     .map_err(|e| e.0)
     .unwrap();
 (aes_dma, output, input) = transfer.wait();
```

## ISR Callback Changes

Previously callbacks were of type `extern "C" fn()`, now they are `IsrCallback`. In most places no changes are needed but when using `bind_interrupt` directly
you need to adapt the code.

```diff
#[esp_hal::ram]
extern "C" fn software3_interrupt() {
    // ...
}

esp_hal::interrupt::bind_interrupt(
    esp_hal::peripherals::Interrupt::FROM_CPU_INTR3,
-   software3_interrupt,
+   IsrCallback::new(software3_interrupt),
);
```

## RMT changes

`PulseCode` used to be an extension trait implemented on `u32`. It is now a
newtype struct, wrapping `u32`.
RMT transmit and receive methods accept `impl Into<PulseCode>` and
`impl From<PulseCode>`, respectively, and implementations for
`PulseCode: From<u32>` and `u32: From<PulseCode>` are provided.

The `PulseCode::empty()` method has been renamed to `PulseCode::end_marker()`,
and the same value can also be obtained via `PulseCode::default()`. Either methods
might be more desirable depending on the context to better communicate the meaning
of this value.

Nevertheless, type annotations will require some changes:

```diff
 let rmt = Rmt::new(peripherals.RMT, freq).unrwap();
 let tx_channel = rmt.channel0.configure_tx(peripherals.GPIO1, TxChannelConfig::default());
 let rx_channel = rmt.channel2.configure_rx(peripherals.GPIO2, RxChannelConfig::default());

-let mut tx_data: [u32; 20] = [PulseCode::new(Level::High, 42, Level::Low, 24); 20];
+let mut tx_data: [PulseCode; 20] = [PulseCode::new(Level::High, 42, Level::Low, 24); 20];

-tx_data[tx_data.len() - 1] = PulseCode::empty();
+tx_data[tx_data.len() - 1] = PulseCode::end_marker();

-let mut rx_data: [u32; 20] = [PulseCode::empty(); 20];
+let mut rx_data: [PulseCode; 20] = [PulseCode::default(); 20];

let _ = tx_channel.transmit(&tx_data).wait().unwrap();

let _ = rx_channel.transmit(&mut rx_data).wait().unwrap();
```

## DMA changes

DMA buffers now have a `Final` associated type parameter. For the publicly available buffer, this is `Self`,
so there is no code change necessary in user codebases. Library writes will need to add the type to their
DMA buffer implementations.

```diff
 unsafe impl DmaTxBuffer for MyTxBuf {
     type View = BufView<Self>;
+    type Final = Self;
     // ...
 }
```

If the `Final` type is not `Self`, `fn from_view()` will need to be updated to return `Self::Final`.

## Timer interrupts
```diff
- pub fn enable_interrupt(&mut self, enable: bool) { ... }
+ pub fn listen(&mut self) { ... }
+ pub fn unlisten(&mut self) { ... }
```

## ESP32-S3 PSRAM Configuration

`PsramConfig::core_clock` is now an `Option`.

```diff
let peripherals = esp_hal::init(
    esp_hal::Config::default()
    .with_cpu_clock(esp_hal::clock::CpuClock::max())
    .with_psram(esp_hal::psram::PsramConfig {
        flash_frequency: esp_hal::psram::FlashFreq::FlashFreq20m,
        ram_frequency: esp_hal::psram::SpiRamFreq::Freq40m,
-       core_clock: esp_hal::psram::SpiTimingConfigCoreClock::SpiTimingConfigCoreClock160m,
+       core_clock: Some(esp_hal::psram::SpiTimingConfigCoreClock::SpiTimingConfigCoreClock160m),
        ..Default::default()
        })
    );
```
