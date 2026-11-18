#[path = "../../../esp-hal/src/sdio_slave/state.rs"]
#[allow(dead_code, unexpected_cfgs)]
mod state;

#[allow(dead_code, unexpected_cfgs)]
mod receive;

#[allow(dead_code)]
mod events;

#[path = "../../../examples/peripheral/sdio/sdio_host/src/voltage.rs"]
mod voltage;

mod tests;
