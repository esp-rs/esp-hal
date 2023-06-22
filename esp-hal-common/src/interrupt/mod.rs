//! Interrupt support

#[cfg(riscv)]
pub use riscv::*;
#[cfg(xtensa)]
pub use xtensa::*;

#[cfg(riscv)]
mod riscv;
#[cfg(xtensa)]
mod xtensa;
