//! Cache suspend and resume around ROM flash operations.
//!
//! Dropping cache lines for a flash range needs the MMU table and lives in
//! [`super::mmu`] instead.

use procmacros::ram;

/// Suspends the flash-backed caches for the duration of a ROM flash operation.
pub(super) struct CacheGuard {
    inner: Inner,
}

cfg_select! {
    esp32 => {
        struct Inner {
            pro: bool,
            app: bool,
        }
    }
    any(esp32s2, esp32s3) => {
        struct Inner {
            icache: u32,
            dcache: u32,
        }
    }
    esp32p4 => {
        // External flash/PSRAM is served by L2. L1 caches internal RAM; suspending
        // it stalls instruction fetch from IRAM (`#[ram]`).
        struct Inner {
            l2: u32,
        }
    }
    esp32s31 => {
        struct Inner {
            i0: u32,
            i1: u32,
            d: u32,
        }
    }
    _ => {
        struct Inner {
            autoload: u32,
        }
    }
}

impl CacheGuard {
    /// Disable the branch predictor (it issues cache requests) and suspend the
    /// external-memory caches.
    #[ram]
    pub(super) fn suspend() -> Self {
        #[cfg(soc_cpu_has_branch_predictor)]
        disable_branch_predictor();

        let inner = suspend_caches();
        Self { inner }
    }

    /// Flush every line. Must run while the caches are still off.
    #[cfg(esp32)]
    #[ram]
    pub(super) fn flush_while_off(&self) {
        unsafe {
            if self.inner.pro {
                Cache_Flush_rom(0);
            }
            if self.inner.app {
                Cache_Flush_rom(1);
            }
        }
    }
}

impl Drop for CacheGuard {
    #[ram]
    fn drop(&mut self) {
        resume_caches(&self.inner);

        #[cfg(soc_cpu_has_branch_predictor)]
        enable_branch_predictor();
    }
}

#[cfg(soc_cpu_has_branch_predictor)]
#[inline(always)]
fn disable_branch_predictor() {
    // MHCR (0x7c1): RS, BFE, BTB. Same bits as `soc::enable_branch_predictor`.
    const MHCR_RS: u32 = 1 << 4;
    const MHCR_BFE: u32 = 1 << 5;
    const MHCR_BTB: u32 = 1 << 12;
    unsafe {
        core::arch::asm!("csrrc x0, 0x7c1, {0}", in(reg) MHCR_RS | MHCR_BFE | MHCR_BTB);
    }
}

#[cfg(soc_cpu_has_branch_predictor)]
#[inline(always)]
fn enable_branch_predictor() {
    const MHCR_RS: u32 = 1 << 4;
    const MHCR_BFE: u32 = 1 << 5;
    const MHCR_BTB: u32 = 1 << 12;
    unsafe {
        core::arch::asm!("csrrs x0, 0x7c1, {0}", in(reg) MHCR_RS | MHCR_BFE | MHCR_BTB);
    }
}

unsafe extern "C" {
    cfg_select! {
        esp32 => {
            fn Cache_Flush_rom(cpu: u32);
        }
        any(esp32s2, esp32s3) => {
            fn Cache_Suspend_ICache() -> u32;
            fn Cache_Suspend_DCache() -> u32;
            fn Cache_Resume_ICache(autoload: u32);
            fn Cache_Resume_DCache(autoload: u32);
        }
        esp32p4 => {
            fn Cache_Suspend_L2_Cache() -> u32;
            fn Cache_Resume_L2_Cache(autoload: u32);
        }
        esp32s31 => {
            fn Cache_Suspend_L1_CORE0_ICache() -> u32;
            fn Cache_Suspend_L1_CORE1_ICache() -> u32;
            fn Cache_Suspend_L1_DCache() -> u32;
            fn Cache_Resume_L1_CORE0_ICache(autoload: u32);
            fn Cache_Resume_L1_CORE1_ICache(autoload: u32);
            fn Cache_Resume_L1_DCache(autoload: u32);
        }
        any(esp32c5, esp32c61) => {
            fn Cache_Suspend_Cache() -> u32;
            fn Cache_Resume_Cache(autoload: u32);
        }
        any(esp32c2, esp32c3, esp32c6, esp32h2) => {
            fn Cache_Suspend_ICache() -> u32;
            fn Cache_Resume_ICache(autoload: u32);
        }
        _ => {
            compile_error!("missing flash cache ROM bindings for this chip");
        }
    }
}

#[ram]
fn suspend_caches() -> Inner {
    cfg_select! {
        esp32 => {
            use crate::peripherals::DPORT;
            let dport = DPORT::regs();
            let pro = dport.pro_cache_ctrl().read().pro_cache_enable().bit();
            let app = dport.app_cache_ctrl().read().app_cache_enable().bit();
            // A cache must be idle before it is turned off, see `cache_ll_l1_disable_cache()`.
            if pro {
                while dport.pro_dcache_dbug0().read().pro_cache_state().bits() != 1 {}
                dport
                    .pro_cache_ctrl()
                    .modify(|_, w| w.pro_cache_enable().clear_bit());
            }
            if app {
                while dport.app_dcache_dbug0().read().app_cache_state().bits() != 1 {}
                dport
                    .app_cache_ctrl()
                    .modify(|_, w| w.app_cache_enable().clear_bit());
            }
            // Complete the writes above before the flash MMU is written.
            let _ = dport.pro_cache_ctrl().read();
            Inner { pro, app }
        }
        any(esp32s2, esp32s3) => {
            #[cfg(esp32s3)]
            use crate::peripherals::EXTMEM;
            unsafe {
                let icache = Cache_Suspend_ICache();
                // ROM `Cache_Suspend_I/DCache` on ESP32-S3 does not wait until
                // the cache FSM is idle (`ESP_ROM_HAS_CACHE_SUSPEND_WAITI_BUG`).
                #[cfg(esp32s3)]
                while EXTMEM::regs().cache_state().read().icache_state().bits() != 1 {}
                let dcache = Cache_Suspend_DCache();
                #[cfg(esp32s3)]
                while EXTMEM::regs().cache_state().read().dcache_state().bits() != 1 {}
                Inner { icache, dcache }
            }
        }
        esp32p4 => unsafe {
            Inner {
                l2: Cache_Suspend_L2_Cache(),
            }
        },
        esp32s31 => unsafe {
            Inner {
                i0: Cache_Suspend_L1_CORE0_ICache(),
                i1: Cache_Suspend_L1_CORE1_ICache(),
                d: Cache_Suspend_L1_DCache(),
            }
        },
        any(esp32c5, esp32c61) => unsafe {
            Inner {
                autoload: Cache_Suspend_Cache(),
            }
        },
        _ => unsafe {
            Inner {
                autoload: Cache_Suspend_ICache(),
            }
        },
    }
}

#[ram]
fn resume_caches(inner: &Inner) {
    cfg_select! {
        esp32 => {
            use crate::peripherals::DPORT;
            let dport = DPORT::regs();
            if inner.pro {
                dport
                    .pro_cache_ctrl()
                    .modify(|_, w| w.pro_cache_enable().set_bit());
            }
            if inner.app {
                dport
                    .app_cache_ctrl()
                    .modify(|_, w| w.app_cache_enable().set_bit());
            }
            // The caller returns to flash right away, so make sure that the cache is on again
            // before the next instruction is fetched: reading the register back completes the
            // write.
            let _ = dport.pro_cache_ctrl().read();
        }
        any(esp32s2, esp32s3) => unsafe {
            Cache_Resume_DCache(inner.dcache);
            Cache_Resume_ICache(inner.icache);
        },
        esp32p4 => unsafe { Cache_Resume_L2_Cache(inner.l2) },
        esp32s31 => unsafe {
            Cache_Resume_L1_DCache(inner.d);
            Cache_Resume_L1_CORE1_ICache(inner.i1);
            Cache_Resume_L1_CORE0_ICache(inner.i0);
        },
        any(esp32c5, esp32c61) => unsafe { Cache_Resume_Cache(inner.autoload) },
        _ => unsafe { Cache_Resume_ICache(inner.autoload) },
    }
}
