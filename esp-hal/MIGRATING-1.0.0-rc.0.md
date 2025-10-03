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

### RMT PulseCode changes

`PulseCode` used to be an extension trait implemented on `u32`. It is now a
newtype struct, wrapping `u32` and providing mostly `const` methods.
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

`PulseCode` constructors have also been reworked, now providing

1. `PulseCode::new()` which panics for out-of-range signal lengths,
2. `PulseCode::new_clamped()` which saturates the signal lengths if out of range,
3. `PulseCode::try_new()` which returns `None` if any signal length is out of range.

The old behaviour of truncating the passed in `u16` to 15 bits is not available
anymore; which method is the best replacement will depend on the use case.

### RMT Channel Changes

`rmt::Channel` used to have a `Raw: RawChannelAccess` generic parameter,
which could be either `ConstChannelAccess<Dir, const CHANNEL: u8>` or `DynChannelAccess<Dir>`.
This generic has been erased, effectively always using `DynChannelAccess`.
The corresponding parameter of the transaction structs has been removed as well
(`SingleShotTxTransaction`, `ContinuousTxTransaction`, `RxTransaction`).

Transmit and receive methods are now directly implemented by
`Channel<Dm: DriverMode, Tx>`
and
`Channel<Dm: DriverMode, Rx>`
respectively and the `RxChannel`, `TxChannel`, `RxChannelAsync` and `TxChannelAsync`
traits have been removed.
Several related types that were previously exported have been removed from the
API as well.

```diff
-use esp_hal::rmt::{ConstChannelAccess, DynChannelAccess, RawChannelAccess};
-let mut tx: Channel<Blocking, ConstChannelAccess<Tx, 0>> = rmt.channel0.configure_tx(NoPin, TxChannelConfig::default());
-let mut rx: Channel<Blocking, ConstChannelAccess<Rx, 2>> = rmt.channel2.configure_rx(NoPin, RxChannelConfig::default());
+let mut tx: Channel<Blocking, Tx> = rmt.channel0.configure_tx(NoPin, TxChannelConfig::default());
+let mut rx: Channel<Blocking, Rx> = rmt.channel2.configure_rx(NoPin, RxChannelConfig::default());

-let mut tx: Channel<Blocking, DynChannelAccess<Tx>> = tx.degrade();
-let mut rx: Channel<Blocking, DynChannelAccess<Rx>> = rx.degrade();

-// same for TxChannelAsync, RxChannelAsync
-use esp_hal::rmt::{TxChannel, RxChannel};
-
-let tx_transaction: SingleShotTxTransaction<'_, DynChannelAccess<Tx>, PulseCode> = tx.transmit(&data);
-let rx_transaction: RxTransaction<'_, DynChannelAccess<Rx>, PulseCode> = rx.transmit(&data);
+let tx_transaction: SingleShotTxTransaction<'_, PulseCode> = tx.transmit(&data);
+let rx_transaction: RxTransaction<'_, PulseCode> = rx.transmit(&data);
```

### RMT method changes

The `rmt::Channel::transmit_continuously` and
`rmt::Channel::transmit_continuously_with_loopcount` methods have been merged:

```diff
-let tx_trans0 = tx_channel0.transmit_continuously(&data);
-let tx_trans1 = tx_channel1.transmit_continuously_with_loopcount(&data, count);
+use core::num::NonZeroU16;
+use esp_hal::rmt::LoopCount;
+let tx_trans0 = tx_channel0.transmit_continuously(&data, LoopCount::Infinite);
+let count = NonZeroU16::new(count).unwrap();
+let tx_trans1 = tx_channel1.transmit_continuously(&data, LoopCount::Finite(count));
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

## `I8080` driver pin configuration changes

```diff
- let tx_pins = TxEightBits::new(
-     peripherals.GPIO9,
-     peripherals.GPIO46,
-     peripherals.GPIO3,
-     peripherals.GPIO8,
-     peripherals.GPIO18,
-     peripherals.GPIO17,
-     peripherals.GPIO16,
-     peripherals.GPIO15,
- );
+ let mut i8080 = I8080::new(
     lcd_cam.lcd,
     peripherals.DMA_CH0,
-    tx_pins,
     config,
  )?
- .with_ctrl_pins(peripherals.GPIO0, peripherals.GPIO47);
+ .with_dc(peripherals.GPIO0)
+ .with_wrx(peripherals.GPIO47)
+ .with_data0(peripherals.GPIO9)
+ .with_data1(peripherals.GPIO46)
+ .with_data2(peripherals.GPIO3)
+ .with_data3(peripherals.GPIO8)
+ .with_data4(peripherals.GPIO18)
+ .with_data5(peripherals.GPIO17)
+ .with_data6(peripherals.GPIO16)
+ .with_data7(peripherals.GPIO15);
```

## I2S Changes

I2S configuration is now done using `i2s::master::Config`. Sample rate and data format, previously passed
to the constructor, have to be assigned to `Config` instead.

```diff
  let i2s = I2s::new(
      peripherals.I2S0,
-     Standard::Philips,
-     DataFormat::Data16Channel16,
-     Rate::from_hz(44100),
      dma_channel,
+     Config::new_tdm_philips()
+         .with_data_format(DataFormat::Data16Channel16)
+         .with_sample_rate(Rate::from_hz(44100)),
  );
```

## RTC Clocks

All RTC clock enums/structs have been moved from `rtc_cntl` to the `clock` module. This affects:

- `RtcClock`
- `RtcFastClock`
- `RtcSlowClock`

Imports will need to be updated accordingly.

Additionally, enum variant naming violations have been resolved, so the `RtcFastClock` and `RtcSlowClock` prefixes will need to be removed from any variants from these enums.

## RISC-V interrupt direct vectoring changes

`enable_direct` now requires user to pass handler function to it.

```diff
interrupt::enable_direct(
    Interrupt::FROM_CPU_INTR0,
    Priority::Priority3,
    CpuInterrupt::Interrupt20,
+   interrupt_handler,
)
.unwrap();
```

## Async/embassy changes

> This section affects `esp-hal-embassy` users. Ariel-OS users are not affected by these changes.

The `esp-hal-embassy` has been discontinued. Embassy is continued to be supported as part of `esp-rtos`.

### Configuration

`esp-hal-embassy` configuration options have not been ported to `esp-rtos`. `esp-rtos` by default works with a single integrated timer queue.
To keep using generic timer queues, use the configuration options provided by `embassy-time`. Multiple timer queues (i.e. the previous `multiple-integrated` option) are not supported.

The `low-power-wait` configuration can be substituted with a custom idle hook. You can specify an idle hook by calling `esp_rtos::start_with_idle_hook`, with a function that just `loop`s.

### Setup

The previous `esp_hal_embassy::main` macro has been replaced by `esp_rtos::main`. The `esp_hal_embassy::init` function has been replaced by `esp_rtos::start`, with a different signature; this function should be used for `esp_radio` as well.

`esp_rtos::start` has a different signature for the different CPU architectures. The function takes a single timer instead of a variable number of timers.

On Xtensa devices (ESP32/S2/S3):

```diff
-#[esp_hal_embassy::main]
+#[esp_rtos::main]
 async fn main(spawner: Spawner) {
     // ... timer setup not shown here.
-    esp_hal_embassy::init([timer0, timer1]);
+    esp_rtos::start(timer0);
     // ...
 }
```

On RISC-V devices (ESP32-C2/C3/C6/H2) you'll need to also pass `SoftwareInterrupt<0>` to `esp_rtos::start`:

```diff
+use esp_hal::interrupt::software::SoftwareInterruptControl;

-#[esp_hal_embassy::main]
+#[esp_rtos::main]
 async fn main(spawner: Spawner) {
     // ... timer setup not shown here.
-    esp_hal_embassy::init([timer0, timer1]);

+    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
+    esp_rtos::start(timer0, software_interrupt.software_interrupt0);
     // ...
 }
```

### Multi-core support

`esp_rtos::embassy::Executor` expects to be run in an `esp_rtos` thread. This means it cannot be started on the second core, unless the second core is managed by `esp_rtos`:

```rust
use esp_hal::system::Stack;
use esp_rtos::embassy::Executor;
use static_cell::StaticCell;

static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();
let app_core_stack = APP_CORE_STACK.init(Stack::new());

// AFTER esp_rtos::start

esp_rtos::start_second_core(
    peripherals.CPU_CTRL,
    sw_int.software_interrupt0,
    sw_int.software_interrupt1,
    app_core_stack,
    move || {
        static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor = EXECUTOR.init(Executor::new());
        executor.run(|spawner| {
            // Spawn tasks from here.
        });
    },
);
```

### Interrupt executor changes

Interrupt executors are provided as `esp_rtos::embassy::InterruptExecutor` with no additional changes.

