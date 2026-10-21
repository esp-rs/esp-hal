//! Branch predictor control.
//!
//! The branch predictor starts cache requests of its own, so it must be off while the cache is
//! off.

/// Branch-predictor bits in `mhcr` (CSR `0x7c1`): return stack, predictive jump, and branch
/// target buffer.
const MHCR_BRANCH_PREDICTOR: u32 = (1 << 4) | (1 << 5) | (1 << 12);

/// Turns the branch predictor on.
#[inline(always)]
pub(crate) fn enable() {
    unsafe {
        core::arch::asm!(
            "fence iorw, iorw",
            "csrs 0x7c1, {0}",
            in(reg) MHCR_BRANCH_PREDICTOR,
        );
    }
}

#[cfg(any(feature = "unstable", all(multi_core, feature = "rt")))]
pub(crate) use guard::DisabledBranchPredictor;

#[cfg(any(feature = "unstable", all(multi_core, feature = "rt")))]
mod guard {
    use super::MHCR_BRANCH_PREDICTOR;

    /// Turns the branch predictor off. [`Drop`] sets the bits that were set before.
    pub(crate) struct DisabledBranchPredictor(u32);

    impl DisabledBranchPredictor {
        /// Turns the branch predictor off.
        #[inline(always)]
        pub(crate) fn new() -> Self {
            let bits: u32;
            unsafe {
                core::arch::asm!(
                    "csrrc {0}, 0x7c1, {1}",
                    "fence iorw, iorw",
                    out(reg) bits,
                    in(reg) MHCR_BRANCH_PREDICTOR,
                );
            }
            Self(bits & MHCR_BRANCH_PREDICTOR)
        }
    }

    impl Drop for DisabledBranchPredictor {
        #[inline(always)]
        fn drop(&mut self) {
            unsafe {
                core::arch::asm!(
                    "fence iorw, iorw",
                    "csrs 0x7c1, {0}",
                    in(reg) self.0,
                );
            }
        }
    }
}
