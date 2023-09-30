
## Embassy Examples

* 😁 Decision Point1: choose systick vs timg0 
- systick is for ARM cortex-m, the esp-hal provides a simple 24-bit clear on write decrementing wrap-on-zero counter for os-scheduling sed for basic countdown timer, interrpt generation on reaching zero. Better for ARM, etc. compatibility.
```
cargo run --example embassy_hello_world --features "embassy","embassy-time-systick"
```


- timg0 is a peripheral esp32-c3 / espressif specific, has more features compared to systick, configurable frequency, has input capture, output compare, can trigger other peripherals or DMA, Suitable for more complex timing requirements like waveform generation, event timing, pulse counting, etc.  there is also a timg1
```
cargo run --example embassy_hello_world --features "embassy","embassy-time-timg0"
```

### examples/embassy_hello_world.rs

examples/embassy_i2c.rs
examples/embassy_i2s_read.rs
examples/embassy_i2s_sound.rs
examples/embassy_rmt_rx.rs
examples/embassy_rmt_tx.rs
examples/embassy_serial.rs
examples/embassy_spi.rs
examples/embassy_wait.rs


