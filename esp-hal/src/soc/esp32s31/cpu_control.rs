use crate::{
    peripherals::{HP_SYS, HP_SYS_CLKRST, LP_AON_CLK_RST, PMU},
    system::Cpu,
};

pub(crate) unsafe fn internal_park_core(core: Cpu, park: bool) {
    // 0x86 = stalled, 0xFF = running (matches IDF cpu_utility_ll.h).
    let code: u8 = if park { 0x86 } else { 0xFF };
    PMU::regs().cpu_stall_sw().modify(|_, w| unsafe {
        match core {
            Cpu::ProCpu => w.hpcore0_sw_stall_code().bits(code),
            Cpu::AppCpu => w.hpcore1_sw_stall_code().bits(code),
        }
    });

    // IDF waits for the core-stalled status to clear after unstalling.
    if !park {
        let status = HP_SYS::regs().cpu_corestalled_st();
        match core {
            Cpu::ProCpu => while status.read().reg_core0_corestalled_st().bit_is_set() {},
            Cpu::AppCpu => while status.read().reg_core1_corestalled_st().bit_is_set() {},
        }
    }
}

pub(crate) fn is_running(core: Cpu) -> bool {
    let stall = PMU::regs().cpu_stall_sw().read();
    let code = match core {
        Cpu::ProCpu => stall.hpcore0_sw_stall_code().bits(),
        Cpu::AppCpu => stall.hpcore1_sw_stall_code().bits(),
    };
    if code == 0x86 {
        return false;
    }

    match core {
        Cpu::ProCpu => true,
        Cpu::AppCpu => {
            let control = HP_SYS_CLKRST::regs().hpcore1_ctrl0().read();
            control.core1_cpu_clk_en().bit_is_set() && control.core1_global_rst_en().bit_is_clear()
        }
    }
}

pub(crate) fn pre_system_reset() {
    // Match IDF's esp_restart_noos(): reset and stall only the other core.
    // The caller must remain alive to request the subsequent system reset.
    let other_core = match Cpu::current() {
        Cpu::ProCpu => Cpu::AppCpu,
        Cpu::AppCpu => Cpu::ProCpu,
    };
    match other_core {
        Cpu::ProCpu => LP_AON_CLK_RST::regs()
            .hpcore0_reset_ctrl()
            .modify(|_, w| w.hpcore0_sw_reset().set_bit()),
        Cpu::AppCpu => LP_AON_CLK_RST::regs()
            .hpcore1_reset_ctrl()
            .modify(|_, w| w.hpcore1_sw_reset().set_bit()),
    };
    unsafe { internal_park_core(other_core, true) };

    // Prevent ROM from jumping to an entry point from the previous image.
    crate::rom::ets_set_appcpu_boot_addr(0);
}

/// Releases core 1 from reset after a CPU-domain power-down wake.
///
/// esp-idf keys this on `core_id == 0` in both the initiator and the helper paths.
#[cfg(all(
    cpu_retention = "software",
    multi_core,
    supports_cpu_power_down,
    feature = "rt"
))]
#[crate::ram]
pub(crate) fn restart_core1_after_wake() {
    if Cpu::current() != Cpu::ProCpu {
        return;
    }
    HP_SYS_CLKRST::regs().hpcore1_ctrl0().modify(|_, w| {
        w.core1_cpu_clk_en()
            .set_bit()
            .core1_global_rst_en()
            .clear_bit()
    });
}

pub(crate) fn disable_core1() {
    // ESP-IDF single-core mode disables both Core 1 clocks and holds the core
    // in global reset.
    HP_SYS_CLKRST::regs().hpcore1_ctrl0().modify(|_, w| {
        w.core1_cpu_clk_en()
            .clear_bit()
            .core1_clic_clk_en()
            .clear_bit()
            .core1_global_rst_en()
            .set_bit()
    });
}

macro_rules! read_csr {
    ($csr:literal) => {{
        let value: u32;
        unsafe { core::arch::asm!(concat!("csrr {0}, ", $csr), out(reg) value) };
        value
    }};
}

macro_rules! write_csr {
    ($csr:literal, $value:expr) => {
        unsafe { core::arch::asm!(concat!("csrw ", $csr, ", {0}"), in(reg) $value) }
    };
}

macro_rules! pma_entries {
    ($($i:literal: $cfg:literal, $addr:literal;)*) => {
        #[cfg(feature = "unstable")]
        pub(crate) fn save_core0_pma(dst: &mut [u32; 32]) {
            $(
                dst[$i] = read_csr!($cfg);
                dst[$i + 16] = read_csr!($addr);
            )*
        }

        #[cfg(feature = "unstable")]
        pub(crate) fn set_core1_pma(src: &[u32; 32]) {
            $(
                write_csr!($addr, src[$i + 16]);
                write_csr!($cfg, src[$i]);
            )*
        }
    };
}

pma_entries! {
    0: "0xbc0", "0xbd0";
    1: "0xbc1", "0xbd1";
    2: "0xbc2", "0xbd2";
    3: "0xbc3", "0xbd3";
    4: "0xbc4", "0xbd4";
    5: "0xbc5", "0xbd5";
    6: "0xbc6", "0xbd6";
    7: "0xbc7", "0xbd7";
    8: "0xbc8", "0xbd8";
    9: "0xbc9", "0xbd9";
    10: "0xbca", "0xbda";
    11: "0xbcb", "0xbdb";
    12: "0xbcc", "0xbdc";
    13: "0xbcd", "0xbdd";
    14: "0xbce", "0xbde";
    15: "0xbcf", "0xbdf";
}

#[cfg(feature = "unstable")]
pub(crate) fn start_core1(entry_point: *const u32) {
    // Enable both clocks and release Core 1 from global reset.
    HP_SYS_CLKRST::regs().hpcore1_ctrl0().modify(|_, w| {
        w.core1_cpu_clk_en()
            .set_bit()
            .core1_clic_clk_en()
            .set_bit()
            .core1_global_rst_en()
            .clear_bit()
    });

    // Core 1's ROM waits for this address before handing control to the app.
    crate::rom::ets_set_appcpu_boot_addr(entry_point as u32);
}
