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
