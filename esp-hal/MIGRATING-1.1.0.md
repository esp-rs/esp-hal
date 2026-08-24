# Migration Guide from 1.1.0 to {{currentVersion}}

## `Clocks` struct removed

The `Clocks` struct and `Clocks::get()` have been removed. The clock frequencies
they provided are now available as lockless free functions from `esp_hal::clock`
and (for chip-specific clocks) `esp_hal::clock::ll`.

| Old | New |
|---|---|
| `Clocks::get().cpu_clock` | `clock::cpu_clock()` |
| `Clocks::get().apb_clock` | `Rate::from_hz(clock::ll::apb_clk_frequency())` |
| `Clocks::get().xtal_clock` | `clock::xtal_clock()` |

```diff
-use esp_hal::clock::Clocks;
+use esp_hal::clock;

-let cpu_freq = Clocks::get().cpu_clock;
-let apb_freq = Clocks::get().apb_clock;
-let xtal_freq = Clocks::get().xtal_clock;
+let cpu_freq = clock::cpu_clock();
+let apb_freq = Rate::from_hz(clock::ll::apb_clk_frequency());
+let xtal_freq = clock::xtal_clock();
```

## Merged DMA-driven SPI drivers

`SpiDmaBus` no longer exists. `SpiDma::with_buffers` now returns `SpiDma` itself,
which implements `embedded_hal::spi::SpiBus` and `embedded_hal_async::spi::SpiBus`
directly.

```diff,rust
-let mut spi: SpiDmaBus<_> = spi_dma.with_buffers(dma_rx_buf, dma_tx_buf);
+let mut spi: SpiDma<_> = spi_dma.with_buffers(dma_rx_buf, dma_tx_buf);
```

Several methods on `SpiDma` were renamed to avoid conflicts with the `SpiBus` trait:

| Before | After |
|---|---|
| `read` | `read_buffer` |
| `write` | `write_buffer` |
| `transfer` | `transfer_buffers` |
| `half_duplex_read` | `half_duplex_read_buffer` |
| `half_duplex_write` | `half_duplex_write_buffer` |

`SpiDmaBus::split` is no longer available; to recover the buffers, use
`SpiDmaTransfer::wait` on the transfer returned by the buffer methods.

## Software interrupts

### `SoftwareInterruptControl` has been removed

Use `FROM_CPU_INTRn` singletons to create `SoftwareInterrupt` instances when needed.

## GPIO

### `LpPinWithResistors` is replaced by `LpPin`

The trait held the pull resistor controls of a low-power pad. Those are internal to `esp-hal` now, so
one trait covers every low-power pad. Name `LpPin` in a bound or a trait object where you named
`LpPinWithResistors`:

```diff
-let wakeup_pins: &mut [(&mut dyn LpPinWithResistors, Level)] = &mut [
+let wakeup_pins: &mut [(&mut dyn LpPin, Level)] = &mut [
     (&mut peripherals.GPIO2, Level::Low),
     (&mut peripherals.GPIO3, Level::High),
 ];

 let ext1 = Ext1WakeupSource::new(wakeup_pins);
```

Use `Input` or `Flex` to set the pull resistors of a pad. A wake pad keeps the resistors that its
driver configured.

## Sleep

### Wakeup sources are enabled through the driver that owns the hardware

A sleep call no longer takes wakeup sources. Each driver enables the source it owns, and the enable
stands until the driver disables it - across a sleep, across a deep-sleep wake, and across the
driver's `Drop`. A sleep that no enabled source could end is refused: `sleep_light` returns without
sleeping and `sleep_deep` panics.

The sleep configuration, which used to be implied by the call, is now a parameter, so a caller who
wants the old behaviour passes `RtcSleepConfig::deep()` or `RtcSleepConfig::default()`.

```rust
// Before
let timer = TimerWakeupSource::new(Duration::from_secs(5));
let mut lpwr = LowPower::new(peripherals.LPWR);
lpwr.sleep_deep(&[&timer]);

// After
let mut lpwr = LowPower::new(peripherals.LPWR);
lpwr.set_wakeup_deadline(Instant::now() + Duration::from_secs(5));
lpwr.sleep_deep(RtcSleepConfig::deep());
```

The deadline is absolute, so the time spent between arming it and sleeping does not shorten the sleep.
It is also standing: the wake it causes does not disarm it, a later call replaces it, and
`clear_wakeup_deadline` ends it.

`sleep_deep` cannot report a rejected sleep, because it does not return. Use
`sleep_deep_with_rejection`, which returns only when the hardware rejects the request because a wakeup
source is already asserted.

### A listening pin wakes the chip

`Input::listen`, `Input::wait_for` and the `wait_for` family arm a light-sleep wake as well as the
interrupt, so `wakeup_enable` and `WaitForOptions` are gone. **A pin that is not listening is not a
wakeup source.**

```rust
// Before
input.wakeup_enable(true, WakeEvent::LowLevel)?;

// After
input.listen(Event::LowLevel);
```

Waking while the high-performance GPIO peripheral is powered down - which every deep sleep does, and a
light sleep does when the caller asks for `pd_hp_periph` or `pd_top` - needs a low-power path, which
only low-power pads have and which the pin has to ask for:

```rust
input.apply_wakeup_config(&WakeupConfig::default().with_low_power_path(true))?;
input.listen(Event::LowLevel);
```

A wake pin keeps the pull resistors it is configured with, and sleep adds none of its own, so give a
level-triggered wake pin a pull against the level it wakes on, or an external resistor. On esp32c2 and
esp32c3, `RtcioWakeupSource` used to pull the pad for you.

`Ext0WakeupSource`, `Ext1WakeupSource` and `RtcioWakeupSource` are gone with no replacement, because
the choice between those paths is no longer the caller's: sleep entry sees every armed pin at once and
allocates the paths for the whole set, which is both optimal and independent of the order the pins were
configured in. It panics if the set cannot be served.

`WakeupLevel` is gone with them: a level-triggered wake now takes its level from the `gpio::Event`
you pass to `listen`, so `WakeupLevel::High` becomes `Event::HighLevel` and `WakeupLevel::Low`
becomes `Event::LowLevel`.

An edge trigger is armed as the level the edge ends on, so a pin listening for `RisingEdge` wakes the
chip on a high level, and one listening for `FallingEdge` on a low level. `AnyEdge` samples the pin at
sleep entry and waits for the opposite level. A pin already at its wake level rejects the sleep instead
of sleeping through the event.

### The low-power cores enable their own wake

```rust
// Before, on esp32s2 and esp32s3
lpwr.sleep_deep(&[&UlpWakeupSource::new()]);

// After
ulp_core.enable_wakeup(WakeupConfig::default());
lpwr.sleep_deep(RtcSleepConfig::deep());
```

```rust
// Before, on esp32c6
lpwr.sleep_deep(&[&WakeFromLpCoreWakeupSource::new()]);

// After
lp_core.enable_wakeup();
lpwr.sleep_deep(RtcSleepConfig::deep());
```

`UlpWakeupSource::set_clear_interrupts_on_sleep` is gone. `UlpCore::run` clears the latched wake
interrupts as it starts the core, the way ESP-IDF does, so nothing chooses.

### UART wake moves onto the UART driver

The threshold is now the number of RX rising edges the chip waits for, the way ESP-IDF counts them,
rather than the raw register value, so the same number wakes the chip on every chip.

```rust
// Before
let uart_wake = Uart0WakeupSource::new(3);
lpwr.sleep_light(&[&uart_wake]);

// After
uart.enable_wakeup(&uart::WakeupConfig::default().with_rising_edges(3))?;
lpwr.sleep_light(RtcSleepConfig::default());
```

### `WakeupReason` is now a set of `WakeupSource`

`WakeupReason` is no longer a `bitflags` type. Wakeup causes are now modelled by the
`WakeupSource` enum, and `WakeupReason` is a thin wrapper around a set of them.

- To iterate over all sources that caused the wakeup, use `wakeup_cause().iter()`.
- Replace `WakeupReason::<Flag>` constants with `WakeupSource::<Flag>` and query the
  set instead of comparing bits:

```diff
-if wakeup_cause() == WakeupReason::NoSleep { /* ... */ }
+if wakeup_cause().is_empty() { /* ... */ }

-if wakeup_cause().contains(WakeupReason::Timer) { /* ... */ }
+if wakeup_cause().contains(WakeupSource::Timer) { /* ... */ }
```

### The `WakeSource` trait and `WakeTriggers` are removed

Custom wakeup sources are no longer part of the public API. Each driver arms the wake for the
hardware it owns, so there is nothing to implement `WakeSource` for and no `WakeTriggers` to fill in.
Drop any custom `WakeSource` implementation and arm the wake through the owning driver instead.

## I2S driver

### TDM configuration moved to `TdmConfig`
TDM builder methods such as `with_sample_rate`, `with_channels`, and `with_data_format` are on `TdmConfig`, not on a public `Config` type.
Update TDM call sites from:
```rust
I2s::new(
peripherals.I2S0,
dma,
Config::new_tdm_philips()
.with_sample_rate(Rate::from_hz(16_000))
.with_data_format(DataFormat::Data16Channel16),
)?;
```
to:
```rust
I2s::new(
 peripherals.I2S0,
 dma,
 TdmConfig::new_tdm_philips()
 .with_sample_rate(Rate::from_hz(16_000))
 .with_data_format(DataFormat::Data16Channel16),
)?;
```

`I2s::new` accepts `TdmConfig` only. Use `I2s::new_pdm` for PDM mode.

### Using PDM mode

PDM is I2S0-only and simplex (TX **or** RX). Use `I2s::new_pdm` with `PdmConfig::tx_only(...)` or `PdmConfig::rx_only(...)`, and connect pins via `with_clk` plus `with_dout` / `with_din` (or `with_din_line` on multi-line chips).
PDM validation errors are returned as `ConfigError::Pdm(PdmError)` (e.g. `PdmError::InvalidLine` for invalid data line indices).

## DMA memory-to-memory

### `MEM2MEM*` peripherals removed

On ESP32-C5/C6/C61/H2 and similar chips, drop `peripherals.MEM2MEMn` from `Mem2Mem::new`. Pass only a channel with `mem2mem = true`, e.g. `Mem2Mem::new(peripherals.DMA_CH0)`.

### `Mem2Mem::new` channel requirements

The channel must implement `Mem2MemCapableChannel` (metadata `mem2mem = true`). Arbitrary `DmaChannel` types no longer work. On ESP32-C3/S3, keep passing a real DMA peripheral (e.g. `SPI2`) as the second argument.

## DMA

### Erased channel types were renamed

The erased (any-channel) DMA types have been renamed to consistently follow the
`{EngineName}Channel` pattern:

| Before | After |
|---|---|
| `AnyGdmaChannel` | `AhbGdmaChannel` |
| `AnyGdmaRxChannel` | `AhbGdmaRxChannel` |
| `AnyGdmaTxChannel` | `AhbGdmaTxChannel` |
| `AnyI2sDmaChannel` | `I2sDmaChannel` |
| `AnyI2sDmaRxChannel` | `I2sDmaRxChannel` |
| `AnyI2sDmaTxChannel` | `I2sDmaTxChannel` |
| `AnySpiDmaChannel` | `SpiDmaChannel` |
| `AnySpiDmaRxChannel` | `SpiDmaRxChannel` |
| `AnySpiDmaTxChannel` | `SpiDmaTxChannel` |

### DMA channel type erasure

`DmaChannelConvert::degrade()` has been removed. Replace calls with `Into`/`From`:

```rust
// Before
let erased = channel.degrade();

// After
let erased = channel.into();
```

### Misc types

`DmaEligible`, `DmaChannelFor`, `RxChannelFor`, `TxChannelFor`, and the
`PeripheralDmaChannel`/`PeripheralRxChannel`/`PeripheralTxChannel` type aliases have been removed.
Driver channel bounds are now expressed directly as the driver-specific trait (e.g.
`SpiDmaChannel`, `I2sDmaChannel`). Update any channel bounds accordingly.

</details>
