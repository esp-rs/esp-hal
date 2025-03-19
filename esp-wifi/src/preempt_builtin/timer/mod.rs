use esp_hal::sync::Locked;

#[cfg_attr(xtensa, path = "xtensa.rs")]
#[cfg_attr(riscv, path = "riscv.rs")]
mod arch_specific;

pub(crate) use arch_specific::*;

use crate::TimeBase;

pub(crate) static TIMER: Locked<Option<TimeBase>> = Locked::new(None);
