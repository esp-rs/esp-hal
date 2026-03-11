use core::ops::Deref;

use super::*;

// RNG must be marked `virtual` for this to work.
impl RNG<'_> {
    /// Return a reference to the register block
    #[inline(always)]
    #[instability::unstable]
    pub const fn regs<'a>() -> &'a RngRegisterBlock {
        &RngRegisterBlock
    }

    /// Return a reference to the register block
    #[inline(always)]
    #[instability::unstable]
    pub fn register_block(&self) -> &RngRegisterBlock {
        &RngRegisterBlock
    }
}

/// Register block overlay for the RNG peripheral
#[instability::unstable]
pub struct RngRegisterBlock;

// This Deref makes it possible to use the rest of the registers.
impl Deref for RngRegisterBlock {
    type Target = pac::rng::RegisterBlock;

    fn deref(&self) -> &Self::Target {
        unsafe { &*pac::RNG::ptr() }
    }
}

impl RngRegisterBlock {
    /// Random number data
    #[instability::unstable]
    pub fn data(&self) -> &pac::rng::DATA {
        let ptr = unsafe { pac::RNG::steal().data() as *const pac::rng::DATA };
        if crate::soc::chip_revision_above(102) {
            // On H2-ECO5+ the LPPERI peripherals contains an additional register inserted before
            // the `rng_data` register.
            // https://github.com/espressif/esp-idf/commit/4c5e1a03414a6d55be4b42ba071b30ad228414f6#diff-bc8f2eca37e32ee4ba21ac812e4934998e764132a400479c4d091eb6f7e2e444
            unsafe { &*ptr.add(1) }
        } else {
            unsafe { &*ptr }
        }
    }
}
