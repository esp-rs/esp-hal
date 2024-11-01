# Migration Guide from 0.21.x to v0.22.x

## Removed `i2s` traits

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

- `I2s`
- `master::SpiDma` and `master::SpiDmaBus`

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

## I2C constructor changes

The `with_timeout` constructors have been removed in favour of `set_timeout` or `with_timeout`.

```diff
-let i2c = I2c::new_with_timeout(peripherals.I2C0, io.pins.gpio4, io.pins.gpio5, 100.kHz(), timeout);
+let i2c = I2c::new(peripherals.I2C0, io.pins.gpio4, io.pins.gpio5, 100.kHz()).with_timeout(timeout);
```

## Changes to half-duplex SPI

The `HalfDuplexMode` and `FullDuplexMode` type parameters have been removed from SPI master and slave
drivers. It is now possible to execute half-duplex and full-duplex operations on the same SPI bus.

### Driver construction

- The `Spi::new_half_duplex` constructor has been removed. Use `new` (or `new_typed`) instead.
- The `with_pins` methods have been removed. Use the individual `with_*` functions instead.
- The `with_mosi` and `with_miso` functions now take input-output peripheral signals to support half-duplex mode.
  > TODO(danielb): this means they are currently only usable with GPIO pins, but upcoming GPIO changes should allow using any output signal.

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

## Circular DMA transfer's `available` returns `Result<usize, DmaError>` now

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
