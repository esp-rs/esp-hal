#[path = "../../../esp-hal/src/sdio_slave/state.rs"]
#[allow(dead_code, unexpected_cfgs)]
mod state;

#[path = "../../../esp-hal/src/sdio_slave/receive.rs"]
#[allow(dead_code, unexpected_cfgs)]
mod receive;

#[path = "../../../esp-hal/src/sdio_slave/events.rs"]
#[allow(dead_code)]
mod events;

#[path = "../../../examples/peripheral/sdio/sdio_host/src/voltage.rs"]
mod voltage;

mod tests;
