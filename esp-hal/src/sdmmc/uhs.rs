//! UHS-I 1.8 V voltage switch.
//!
//! Implements the DesignWare `volt_switch` handshake for CMD11 and tracks
//! whether the bus is really at 1.8 V. Only compiled for chips whose SD host
//! supports the switch (`sdmmc_uhs`).

use core::sync::atomic::AtomicBool;

use super::*;

/// Set when the SD IO rail is at 1.8 V, either because CMD11 completed or
/// because the deferred rail move restored it after a warm reset. Only then may
/// the card clock go past the 3.3 V ceiling (50 MHz High Speed); 100 MHz is
/// only in spec at 1.8 V.
pub(super) static UHS_1V8_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Sets or clears the controller's per-slot 1.8 V bit.
///
/// The P4 register description says this bit does nothing (the rail is
/// controlled through the LDO API), but esp-idf still maintains it for parity
/// with the DesignWare controller, and other `sdmmc_uhs` chips may use it.
pub(super) fn set_uhs_volt(slot: SlotId, enabled: bool) {
    let bit = 1u32 << slot.index();
    SDHOST::regs().uhs().modify(|rd, w| unsafe {
        let bits = if enabled {
            rd.bits() | bit
        } else {
            rd.bits() & !bit
        };
        w.bits(bits)
    });
}

/// Clears every slot's 1.8 V bit, e.g. when the shared rail is powered off.
pub(super) fn clear_uhs_volt_all() {
    SDHOST::regs().uhs().modify(|_, w| unsafe { w.bits(0) });
}

/// `RINTSTS` bit 10: the DesignWare voltage-switch completion interrupt.
///
/// It is aliased onto HTO (`SDMMC_INTMASK_VOLT_SW == SDMMC_INTMASK_HTO`). While
/// a `volt_switch` command runs the *controller* raises it -- never the card,
/// and never CDONE.
const EVT_VOLT_SW: u32 = EVT_HTO;

// `CMD` register field bits that the PAC does not model, used only by the
// UHS-I 1.8 V switch sequence.
const CMD_CARD_NUMBER_SHIFT: u32 = 16;
const CMD_WAIT_PRVDATA_COMPLETE: u32 = 1 << 13;
const CMD_UPDATE_CLOCK_REGISTERS_ONLY: u32 = 1 << 21;
const CMD_VOLT_SWITCH: u32 = 1 << 28;
const CMD_USE_HOLD_REG: u32 = 1 << 29;
const CMD_START: u32 = 1 << 31;
const CMD_RESPONSE_EXPECT: u32 = 1 << 6;
const CMD_CHECK_RESPONSE_CRC: u32 = 1 << 8;

/// esp-idf `sdmmc_host_clock_update_command(slot, is_cmd11)`: push
/// CLKDIV/CLKSRC/CLKENA into the card-clock domain **without sending anything
/// to the card** (CMD bit 21). `volt_switch` (bit 28) turns it into one of the
/// two clock-update steps of the 1.8 V handshake; esp-idf sets it on both and
/// the controller will not complete the sequence without it.
fn clock_update_command(slot: SlotId, volt_switch: bool) -> Result<(), Error> {
    let mut bits = ((slot.index() as u32) << CMD_CARD_NUMBER_SHIFT)
        | CMD_WAIT_PRVDATA_COMPLETE
        | CMD_UPDATE_CLOCK_REGISTERS_ONLY
        | CMD_USE_HOLD_REG
        | CMD_START;
    if volt_switch {
        bits |= CMD_VOLT_SWITCH;
    }

    let r = SDHOST::regs();
    r.cmdarg().write(|w| unsafe { w.bits(0) });
    r.cmd().write(|w| unsafe { w.bits(bits) });
    wait_command_accepted()
}

/// Waits for the voltage-switch completion interrupt (or a command error) and
/// returns the full `rintsts` snapshot.
fn wait_volt_sw(timeout: Duration) -> Result<u32, Error> {
    let r = SDHOST::regs();
    let start = Instant::now();
    loop {
        let sts = r.rintsts().read().bits();
        if sts & (EVT_VOLT_SW | EVT_RTO | EVT_RCRC | EVT_RESP_ERR | EVT_HLE) != 0 {
            return Ok(sts);
        }
        if start.elapsed() >= timeout {
            return Err(Error::Timeout);
        }
        core::hint::spin_loop();
    }
}

/// Best-effort restore of the clock gating and interrupt state if the CMD11
/// handshake exits early.
///
/// The handshake disables low-power clock gating (and, from stage 2 on, the
/// card clock) before the rail move. On an error path the next operation would
/// otherwise run with the clock stopped or `EVT_VOLT_SW` still pending.
///
/// `INTMASK`/`IDINTEN` are left at the idle mask here; the next command or
/// transfer path reprograms them.
struct Cmd11Cleanup {
    slot: SlotId,
    armed: bool,
}

impl Cmd11Cleanup {
    fn new(slot: SlotId) -> Self {
        Self { slot, armed: true }
    }

    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for Cmd11Cleanup {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }

        let r = SDHOST::regs();
        let mask = 1u8 << self.slot.index();
        r.clkena().modify(|rd, w| unsafe {
            w.cclk_enable().bits(rd.cclk_enable().bits() | mask);
            w.lp_enable().bits(rd.lp_enable().bits() | mask)
        });
        // Drop cannot propagate the error; the next command path will report
        // any real failure.
        let _ = clock_update_command(self.slot, false);
        set_uhs_volt(
            self.slot,
            UHS_1V8_ACTIVE.load(core::sync::atomic::Ordering::Relaxed),
        );
        r.rintsts().write(|w| unsafe { w.bits(EVT_VOLT_SW) });
    }
}

/// The SD UHS-I 1.8 V voltage-switch sequence for CMD11, ported from esp-idf
/// (`sd_host_slot_sdmmc_do_transaction` / `handle_voltage_switch_stage{1,2,3}`
/// in `sd_trans_sdmmc.c`, with `handle_voltage_switch_stage{1,2,3}` in
/// `sd_transaction.c` for v5.x). Linux `dw_mmc` and U-Boot `dw_mmc` implement
/// the same controller dance.
///
/// The DesignWare controller drives the handshake, not the host:
///
/// 1. keep the card clock always on during the whole dance -- clear `CLKENA.LOW_PWR` and push it
///    with a plain clock-update command (the low-power auto clock stop would otherwise fire
///    mid-switch and confuse the card);
/// 2. issue CMD11 with `volt_switch` (bit 28). The controller sends the command, captures the R1
///    and then raises the **VOLT_SW** interrupt (it never raises CDONE for a `volt_switch`
///    command);
/// 3. on VOLT_SW: stop the card clock (`cclk_enable = 0`, pushed with a **`volt_switch`** clock
///    update), wait 100 us, move the SD I/O rail to 1.8 V, wait 10 ms for it to settle, then
///    restart the clock (pushed with another **`volt_switch`** clock update);
/// 4. on a second VOLT_SW: restore `CLKENA.LOW_PWR` with a plain clock update.
///
/// Completion is the pair of VOLT_SW interrupts, *not* `STATUS.data_busy`:
/// esp-idf's CMD11 carries no `SCF_WAIT_BUSY` (the card's DAT-low is not the
/// handshake's completion signal on this controller). `UHS.VOLT` is written for
/// parity but has no effect on the rail, which is controlled separately by the
/// chip-specific code.
pub(super) fn cmd11_voltage_switch(slot: SlotId) -> Result<[u32; 4], Error> {
    let r = SDHOST::regs();
    let idx = slot.index();
    // The CLKENA fields are 2 bits wide (u8); UHS/VOLT is a raw u32 write.
    let clk_mask = 1u8 << idx;

    // Restore the clock gating/clock enable and clear a lingering VOLT_SW if
    // any stage below returns early.
    let mut cleanup = Cmd11Cleanup::new(slot);

    // Poll RINTSTS by hand: keep VOLT_SW out of INTMASK so the (card-detect) ISR
    // cannot write-1-clear it before we observe it, and clear any stale copy.
    r.intmask().write(|w| unsafe { w.bits(idle_intmask()) });
    r.rintsts().write(|w| unsafe { w.bits(EVT_VOLT_SW) });
    r.idinten().write(|w| unsafe { w.bits(0) });

    // Keep the card clock always on during the switch.
    r.clkena()
        .modify(|rd, w| unsafe { w.lp_enable().bits(rd.lp_enable().bits() & !clk_mask) });
    clock_update_command(slot, false)?;

    // CMD11 with `volt_switch`; the controller raises VOLT_SW, not CDONE.
    let cmd11 = ((idx as u32) << CMD_CARD_NUMBER_SHIFT)
        | 11
        | CMD_RESPONSE_EXPECT
        | CMD_CHECK_RESPONSE_CRC
        | CMD_USE_HOLD_REG
        | CMD_VOLT_SWITCH
        | CMD_START;
    r.cmdarg().write(|w| unsafe { w.bits(0) });
    r.cmd().write(|w| unsafe { w.bits(cmd11) });
    wait_command_accepted()?;

    // The controller's CMD11 handshake is slow (Linux budgets ~130 ms before the
    // interrupt), so allow a generous 1 s.
    let sts = wait_volt_sw(Duration::from_millis(1_000))?;
    map_rintsts(sts & !EVT_VOLT_SW)?;
    let resp0 = r.resp0().read().bits();

    // Stop the clock, move the rail, restart the clock.
    r.rintsts().write(|w| unsafe { w.bits(EVT_VOLT_SW) });
    r.clkena()
        .modify(|rd, w| unsafe { w.cclk_enable().bits(rd.cclk_enable().bits() & !clk_mask) });
    clock_update_command(slot, true)?;
    crate::rom::ets_delay_us(100);
    chip_specific::set_io_voltage_1v8();
    // The rail is at 1.8 V from here on, even if the rest of the handshake
    // fails (the second VOLT_SW can time out). Recording it now keeps a later
    // init from forcing 3.3 V into a card that already switched.
    UHS_1V8_ACTIVE.store(true, core::sync::atomic::Ordering::Relaxed);
    set_uhs_volt(slot, false);
    crate::rom::ets_delay_us(10_000);
    r.clkena()
        .modify(|rd, w| unsafe { w.cclk_enable().bits(rd.cclk_enable().bits() | clk_mask) });
    clock_update_command(slot, true)?;
    set_uhs_volt(slot, true);

    // Second VOLT_SW: back to low-power clock gating.
    let sts = wait_volt_sw(Duration::from_millis(1_000))?;
    map_rintsts(sts & !EVT_VOLT_SW)?;

    r.rintsts().write(|w| unsafe { w.bits(EVT_VOLT_SW) });
    r.clkena()
        .modify(|rd, w| unsafe { w.lp_enable().bits(rd.lp_enable().bits() | clk_mask) });
    clock_update_command(slot, false)?;

    cleanup.disarm();
    Ok([resp0, 0, 0, 0])
}
