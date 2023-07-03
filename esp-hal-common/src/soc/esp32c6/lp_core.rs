//! Control the LP core

use esp32c6 as pac;

use crate::peripheral::{Peripheral, PeripheralRef};

#[derive(Debug, Clone, Copy)]
pub enum LpCoreWakeupSource {
    HpCpu,
}

pub struct LpCore<'d> {
    _lp_core: PeripheralRef<'d, crate::soc::peripherals::LP_CORE>,
}

impl<'d> LpCore<'d> {
    pub fn new(lp_core: impl Peripheral<P = crate::soc::peripherals::LP_CORE> + 'd) -> Self {
        crate::into_ref!(lp_core);
        Self { _lp_core: lp_core }
    }

    pub fn stop(&mut self) {
        ulp_lp_core_stop();
    }

    pub fn run(&mut self, wakeup_src: LpCoreWakeupSource) {
        ulp_lp_core_run(wakeup_src);
    }
}

fn ulp_lp_core_stop() {
    let pmu = unsafe { &*pac::PMU::PTR };
    pmu.lp_cpu_pwr1
        .modify(|_, w| unsafe { w.lp_cpu_wakeup_en().bits(0) });
    pmu.lp_cpu_pwr1
        .modify(|_, w| w.lp_cpu_sleep_req().set_bit());
}

fn ulp_lp_core_run(wakeup_src: LpCoreWakeupSource) {
    let lp_aon = unsafe { &*pac::LP_AON::PTR };
    let pmu = unsafe { &*pac::PMU::PTR };
    let lp_peri = unsafe { &*pac::LP_PERI::PTR };

    // Enable LP-Core
    lp_aon.lpcore.modify(|_, w| w.disable().clear_bit());

    // Allow LP core to access LP memory during sleep
    lp_aon.lpbus.modify(|_, w| w.fast_mem_mux_sel().clear_bit());
    lp_aon
        .lpbus
        .modify(|_, w| w.fast_mem_mux_sel_update().set_bit());

    // Enable stall at sleep request
    pmu.lp_cpu_pwr0
        .modify(|_, w| w.lp_cpu_slp_stall_en().set_bit());

    // Enable reset after wake-up
    pmu.lp_cpu_pwr0
        .modify(|_, w| w.lp_cpu_slp_reset_en().set_bit());

    // Set wake-up sources
    let src = match wakeup_src {
        LpCoreWakeupSource::HpCpu => 0x01,
    };
    pmu.lp_cpu_pwr1
        .modify(|_, w| w.lp_cpu_wakeup_en().variant(src));

    // Enable JTAG debugging
    lp_peri
        .cpu
        .modify(|_, w| w.lpcore_dbgm_unavaliable().clear_bit());

    // wake up
    match wakeup_src {
        LpCoreWakeupSource::HpCpu => {
            pmu.hp_lp_cpu_comm.write(|w| w.hp_trigger_lp().set_bit());
        }
    }
}
