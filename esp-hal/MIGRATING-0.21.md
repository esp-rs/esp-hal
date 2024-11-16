# Migration Guide from 0.21.x to v0.22.x

## IO changes

### GPIO pins are now accessible via `Peripherals`

```diff
 let peripherals = esp_hal::init(Default::default());
-let io = Io::new(peripherals.GPIO, peripherals.IOMUX);
-let pin = io.pins.gpio5;
+let pin = peripherals.GPIO5;
```

### `Io` constructor changes

- `new_with_priority` and `new_no_bind_interrupts` have been removed.
  Use `set_priority` to configure the GPIO interrupt priority.
  We no longer overwrite interrupt handlers set by user code during initialization.
- `new` no longer takes `peripherals.GPIO`

## Removed `async`-specific constructors

The following async-specific constuctors have been removed:

- `configure_for_async` DMA channel constructors
- `TwaiConfiguration::new_async` and `TwaiConfiguration::new_async_no_transceiver`
- `I2c::new_async`
- `LcdCam::new_async`
- `UsbSerialJtag::new_async`
- `Rsa::new_async`
- `Rmt::new_async`
- `Uart::new_async`, `Uart::new_async_with_config`
- `UartRx::new_async`, `UartRx::new_async_with_config`
- `UartTx::new_async`, `UartTx::new_async_with_config`

You can use the blocking counterparts, then call `into_async` on the returned peripheral instead.

```diff
-let mut config = twai::TwaiConfiguration::new_async(
+let mut config = twai::TwaiConfiguration::new(
     peripherals.TWAI0,
     loopback_pin.peripheral_input(),
     loopback_pin,
     twai::BaudRate::B1000K,
     TwaiMode::SelfTest,
-);
+).into_async();
```

Some drivers were implicitly configured to the asyncness of the DMA channel used to construct them.
This is no longer the case, and the following drivers will always be created in blocking mode:

- `i2s::master::I2s`
- `spi::master::SpiDma` and `spi::master::SpiDmaBus`

## Peripheral types are now optional

You no longer have to specify the peripheral instance in the driver's type for the following
peripherals:

- SPI (both master and slave)
- I2S
- I2C
- TWAI
- UART

```diff
-Spi<'static, SPI2, FullDuplexMode>
+Spi<'static, FullDuplexMode>

-SpiDma<'static, SPI2, HalfDuplexMode, Blocking>
+SpiDma<'static, HalfDuplexMode, Blocking>

-I2sTx<'static, I2S0, Async>
+I2sTx<'static, Async>
```

Note that you may still specify the instance if you need to. To do this, we provide `_typed`
versions of the constructors (for example: `new_typed`, `new_half_duplex_typed`). Please note that
the peripheral instance has been moved to the last generic parameter position.

```rust
let spi: Spi<'static, FullDuplexMode, SPI2> = Spi::new_typed(peripherals.SPI2, 1.MHz(), SpiMode::Mode0);
```

## I2C changes

The I2C master driver and related types have been moved to `esp_hal::i2c::master`.

The `with_timeout` constructors have been removed. `new` and `new_typed` now take a `Config` struct
with the available configuration options.

- The default configuration is now:
  - bus frequency: 100 kHz
  - timeout: about 10 bus clock cycles

The constructors no longer take pins. Use `with_sda` and `with_scl` instead.

```diff
-use esp_hal::i2c::I2c;
+use esp_hal::i2c::{Config, I2c};
-let i2c = I2c::new_with_timeout(peripherals.I2C0, io.pins.gpio4, io.pins.gpio5, 100.kHz(), timeout);
+I2c::new_with_config(
+    peripherals.I2C0,
+    {
+        let mut config = Config::default();
+        config.frequency = 100.kHz();
+        config.timeout = timeout;
+        config
+    },
+)
+.with_sda(io.pins.gpio4)
+.with_scl(io.pins.gpio5);
```

### The calculation of I2C timeout has changed

Previously, I2C timeouts were counted in increments of I2C peripheral clock cycles. This meant that
the configure value meant different lengths of time depending on the device. With this update, the
I2C configuration now expects the timeout value in number of bus clock cycles, which is consistent
between devices.

ESP32 and ESP32-S2 use an exact number of clock cycles for its timeout. Other MCUs, however, use
the `2^timeout` value internally, and the HAL rounds up the timeout to the next appropriate value.

## Changes to half-duplex SPI

The `HalfDuplexMode` and `FullDuplexMode` type parameters have been removed from SPI master and slave
drivers. It is now possible to execute half-duplex and full-duplex operations on the same SPI bus.

### Driver construction

- The `Spi::new_half_duplex` constructor has been removed. Use `new` (or `new_typed`) instead.
- The `with_pins` methods have been removed. Use the individual `with_*` functions instead.
- The `with_mosi` and `with_miso` functions now take input-output peripheral signals to support half-duplex mode.

```diff
- let mut spi = Spi::new_half_duplex(peripherals.SPI2, 100.kHz(), SpiMode::Mode0)
-     .with_pins(sck, mosi, miso, sio2, sio3, cs);
+ let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0)
+     .with_sck(sck)
+     .with_cs(cs)
+     .with_mosi(mosi)
+     .with_miso(miso)
+     .with_sio2(sio2)
+     .with_sio3(sio3);
```

### Transfer functions

The `Spi<'_, SPI, HalfDuplexMode>::read` and `Spi<'_, SPI, HalfDuplexMode>::write` functions have been replaced by
`half_duplex_read` and `half_duplex_write`.

```diff
 let mut data = [0u8; 2];
 let transfer = spi
-    .read(
+    .half_duplex_read(
         SpiDataMode::Single,
         Command::Command8(0x90, SpiDataMode::Single),
         Address::Address24(0x000000, SpiDataMode::Single),
         0,
         &mut data,
     )
     .unwrap();

 let transfer = spi
-    .write(
+    .half_duplex_write(
         SpiDataMode::Quad,
         Command::Command8(write as u16, command_data_mode),
         Address::Address24(
             write as u32 | (write as u32) << 8 | (write as u32) << 16,
             SpiDataMode::Quad,
         ),
         0,
         dma_tx_buf,
     )
     .unwrap();
```

## Slave-mode SPI

### Driver construction

The constructors no longer accept pins. Use the `with_pin_name` setters instead.

```diff
 let mut spi = Spi::new(
     peripherals.SPI2,
-    sclk,
-    mosi,
-    miso,
-    cs,
     SpiMode::Mode0,
-);
+)
+.with_sclk(sclk)
+.with_mosi(mosi)
+.with_miso(miso)
+.with_cs(cs);
```

## UART event listening

The following functions have been removed:

- `listen_at_cmd`
- `unlisten_at_cmd`
- `listen_tx_done`
- `unlisten_tx_done`
- `listen_rx_fifo_full`
- `unlisten_rx_fifo_full`
- `at_cmd_interrupt_set`
- `tx_done_interrupt_set`
- `rx_fifo_full_interrupt_set`
- `reset_at_cmd_interrupt`
- `reset_tx_done_interrupt`
- `reset_rx_fifo_full_interrupt`

You can now use the `UartInterrupt` enum and the corresponding `listen`, `unlisten`, `interrupts` and `clear_interrupts` functions.

Use `interrupts` in place of `<INTERRUPT>_interrupt_set` and `clear_interrupts` in place of the old `reset_` functions.

`UartInterrupt`:

- `AtCmd`
- `TxDone`
- `RxFifoFull`

Checking interrupt bits is now done using APIs provided by [enumset](https://docs.rs/enumset/1.1.5/enumset/index.html). For example, to see if
a particular interrupt bit is set, use `contains`:

```diff
-serial.at_cmd_interrupt_set()
+serial.interupts().contains(UartInterrupt::AtCmd)
```

You can now listen/unlisten multiple interrupt bits at once:

```diff
-uart0.listen_at_cmd();
-uart0.listen_rx_fifo_full();
+uart0.listen(UartInterrupt::AtCmd | UartConterrupt::RxFifoFull);
```

## I2S changes

### The I2S driver has been moved to `i2s::master`

```diff
-use esp_hal::i2s::{DataFormat, I2s, Standard};
+use esp_hal::i2s::master::{DataFormat, I2s, Standard};
```

### Removed `i2s` traits

The following traits have been removed:

- `I2sWrite`
- `I2sWriteDma`
- `I2sRead`
- `I2sReadDma`
- `I2sWriteDmaAsync`
- `I2sReadDmaAsync`

You no longer have to import these to access their respective APIs. If you used these traits
in your functions as generic parameters, you can use the `I2s` type directly instead.

For example:

```diff
-fn foo(i2s: &mut impl I2sWrite) {
+fn foo(i2s: &mut I2s<'_, I2S0, Blocking>) {
     // ...
 }
```

## DMA related changes

### Circular DMA transfer's `available` returns `Result<usize, DmaError>` now

In case of any error you should drop the transfer and restart it.

```diff
     loop {
-        let avail = transfer.available();
+        let avail = match transfer.available() {
+            Ok(avail) => avail,
+            Err(_) => {
+                core::mem::drop(transfer);
+                transfer = i2s_tx.write_dma_circular(&tx_buffer).unwrap();
+                continue;
+            },
+        };
```

### Channel, ChannelRx and ChannelTx types have changed

- `Channel`'s `Async`/`Blocking` mode has been moved before the channel instance parameter.
- `ChannelRx` and `ChannelTx` have gained a new `Async`/`Blocking` mode parameter.

```diff
-Channel<'d, DmaChannel0, Async>
+Channel<'d, Async, DmaChannel0>

-ChannelRx<'d, DmaChannel0>
+ChannelRx<'d, Async, DmaChannel0>

-ChannelTx<'d, DmaChannel0>
+ChannelTx<'d, Async, DmaChannel0>
```

## Removed `peripheral_input` and `into_peripheral_output` from GPIO pin types

Creating peripheral interconnect signals now consume the GPIO pin used for the connection.

The previous signal function have been replaced by `split`. This change affects the following APIs:

- `GpioPin`
- `AnyPin`

```diff
-let input_signal = gpioN.peripheral_input();
-let output_signal = gpioN.into_peripheral_output();
+let (input_signal, output_signal) = gpioN.split();
```

`into_peripheral_output`, `split` (for output pins only) and `peripheral_input` have been added to
the GPIO drivers (`Input`, `Output`, `OutputOpenDrain` and `Flex`) instead.

## ETM changes

- The types are no longer prefixed with `GpioEtm`, `TimerEtm` or `SysTimerEtm`. You can still use
  import aliasses in case you need to differentiate due to name collisions
  (e.g. `use esp_hal::gpio::etm::Event as GpioEtmEvent`).
- The old task and event types have been replaced by `Task` and `Event`.
- GPIO tasks and events are no longer generic.

## Changes to peripheral configuration

### The `uart::config` module has been removed

The module's contents have been moved into `uart`.

```diff
-use esp_hal::uart::config::Config;
+use esp_hal::uart::Config;
```

If you work with multiple configurable peripherals, you may want to import the `uart` module and
refer to the `Config` struct as `uart::Config`.

### SPI drivers can now be configured using `spi::master::Config`

- The old methods to change configuration have been removed.
- The `new` and `new_typed` constructor no longer takes `frequency` and `mode`.
- The default configuration is now:
  - bus frequency: 1 MHz
  - bit order: MSB first
  - mode: SPI mode 0
- There are new constructors (`new_with_config`, `new_typed_with_config`) and a new `apply_config` method to apply custom configuration.

```diff
-use esp_hal::spi::{master::Spi, SpiMode};
+use esp_hal::spi::{master::{Config, Spi}, SpiMode};
-Spi::new(SPI2, 100.kHz(), SpiMode::Mode1);
+Spi::new_with_config(
+    SPI2,
+    Config {
+        frequency: 100.kHz(),
+        mode: SpiMode::Mode0,
+        ..Config::default()
+    },
+)
```

## LCD_CAM changes

### I8080 driver split `set_byte_order()` into `set_8bits_order()` and `set_byte_order()`.

If you were using an 8-bit bus.

```diff
- i8080.set_byte_order(ByteOrder::default());
+ i8080.set_8bits_order(ByteOrder::default());
```

If you were using an 16-bit bus, you don't need to change anything, `set_byte_order()` now works correctly.

If you were sharing the bus between an 8-bit and 16-bit device, you will have to call the corresponding method when
you switch between devices. Be sure to read the documentation of the new methods.

### Mixed mode constructors

It is no longer possible to construct `I8080` or `Camera` with an async-mode DMA channel.
Convert the DMA channel into blocking before passing it to these constructors.

```diff
 let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
 let channel = ctx
     .dma
     .channel0
-    .configure(false, DmaPriority::Priority0)
-    .into_async();
+    .configure(false, DmaPriority::Priority0);

 let i8080 = I8080::new(
     lcd_cam.lcd,
     channel.tx,
     pins,
     20.MHz(),
     Config::default(),
 );
```

## `rmt::Channel::transmit` now returns `Result`, `PulseCode` is now `u32`

When trying to send a one-shot transmission will fail if it doesn't end with an end-marker.

```diff
-    let mut data = [PulseCode {
-        level1: true,
-        length1: 200,
-        level2: false,
-        length2: 50,
-    }; 20];
-
-    data[data.len() - 2] = PulseCode {
-        level1: true,
-        length1: 3000,
-        level2: false,
-        length2: 500,
-    };
-    data[data.len() - 1] = PulseCode::default();
+    let mut data = [PulseCode::new(true, 200, false, 50); 20];
+    data[data.len() - 2] = PulseCode::new(true, 3000, false, 500);
+    data[data.len() - 1] = PulseCode::empty();

-        let transaction = channel.transmit(&data);
+        let transaction = channel.transmit(&data).unwrap();
```


## The `parl_io::NoClkPin` and `no_clk_pin()` have been removed

You can use `gpio::NoPin` instead.

```diff
 use esp_hal:: {
-   parl_io::no_clk_pin,
+   gpio::NoPin,
 }

-parl_io.rx.with_config(&mut rx_pins, no_clk_pin(), BitPackOrder::Msb, Some(0xfff))
+let mut rx_clk_pin = NoPin;
+parl_io.rx.with_config(&mut rx_pins, &mut rx_clk_pin, BitPackOrder::Msb, Some(0xfff))
```

## `get_` prefixes have been removed from functions

In order to better comply with the Rust API Guidelines [getter names convention], we have removed the `get_` prefixes from all functions which previously had it. Due to the number of changes it's not practical to list all changes here, however if a function previous began with `get_`, you can simply remove this prefix.

[getter names convention]: https://rust-lang.github.io/api-guidelines/naming.html#c-getter

## The `get_core()` function has been removed in favour of `Cpu::current()`

```diff
- let core = esp_hal::get_core();
+ let core = esp_hal::Cpu::current();
```
