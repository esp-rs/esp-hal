use enumset::EnumSet;

use super::{
    super::{Config, DataMode, SpiInterrupt},
    Driver,
};
use crate::{
    RegisterToggle,
    spi::{Error, Mode},
};

pub(super) fn abort_transfer(driver: &Driver) {
    // CPU-controlled transfer abort on ESP32 works by briefly toggling slave mode.
    driver.regs().slave().toggle(|w, en| w.mode().bit(en));
}

pub(super) fn init(driver: &Driver) {
    driver.regs().ctrl().modify(|_, w| w.wp().clear_bit());
}

pub(super) fn init_spi_data_mode(
    driver: &Driver,
    cmd_mode: DataMode,
    address_mode: DataMode,
    data_mode: DataMode,
) -> Result<(), Error> {
    match cmd_mode {
        DataMode::Single | DataMode::SingleTwoDataLines => (),
        _ => {
            error!("Commands must be single bit wide");
            return Err(Error::Unsupported);
        }
    }

    match address_mode {
        DataMode::Single | DataMode::SingleTwoDataLines => {
            driver.regs().ctrl().modify(|_, w| {
                w.fastrd_mode()
                    .bit(matches!(data_mode, DataMode::Dual | DataMode::Quad));
                w.fread_dio().clear_bit();
                w.fread_qio().clear_bit();
                w.fread_dual().bit(data_mode == DataMode::Dual);
                w.fread_quad().bit(data_mode == DataMode::Quad)
            });

            driver.regs().user().modify(|_, w| {
                w.fwrite_dio().clear_bit();
                w.fwrite_qio().clear_bit();
                w.fwrite_dual().bit(data_mode == DataMode::Dual);
                w.fwrite_quad().bit(data_mode == DataMode::Quad)
            });
        }
        address_mode if address_mode == data_mode => {
            driver.regs().ctrl().modify(|_, w| {
                w.fastrd_mode()
                    .bit(matches!(data_mode, DataMode::Dual | DataMode::Quad));
                w.fread_dio().bit(address_mode == DataMode::Dual);
                w.fread_qio().bit(address_mode == DataMode::Quad);
                w.fread_dual().clear_bit();
                w.fread_quad().clear_bit()
            });

            driver.regs().user().modify(|_, w| {
                w.fwrite_dio().bit(address_mode == DataMode::Dual);
                w.fwrite_qio().bit(address_mode == DataMode::Quad);
                w.fwrite_dual().clear_bit();
                w.fwrite_quad().clear_bit()
            });
        }
        _ => {
            error!("Address mode must be single bit wide or equal to the data mode");
            return Err(Error::Unsupported);
        }
    }

    Ok(())
}

pub(super) fn enable_listen(driver: &Driver, interrupts: EnumSet<SpiInterrupt>, enable: bool) {
    driver.regs().slave().modify(|_, w| {
        for interrupt in interrupts {
            match interrupt {
                SpiInterrupt::TransferDone => w.trans_inten().bit(enable),
            };
        }
        w
    });
}

pub(super) fn interrupts(driver: &Driver) -> EnumSet<SpiInterrupt> {
    let mut res = EnumSet::new();
    if driver.regs().slave().read().trans_done().bit() {
        res.insert(SpiInterrupt::TransferDone);
    }
    res
}

pub(super) fn clear_interrupts(driver: &Driver, interrupts: EnumSet<SpiInterrupt>) {
    for interrupt in interrupts {
        match interrupt {
            SpiInterrupt::TransferDone => {
                driver
                    .regs()
                    .slave()
                    .modify(|_, w| w.trans_done().clear_bit());
            }
        }
    }
}

pub(super) fn apply_config(driver: &Driver, config: &Config) {
    let f_apb = 80_000_000;
    let source_freq_hz = match config.clock_source {
        super::super::ClockSource::Apb => f_apb,
    };

    let clock_reg = driver.regs().clock().read();
    let eff_clk = if clock_reg.clk_equ_sysclk().bit_is_set() {
        f_apb
    } else {
        let pre = clock_reg.clkdiv_pre().bits() as i32 + 1;
        let n = clock_reg.clkcnt_n().bits() as i32 + 1;
        f_apb / (pre * n)
    };

    let apbclk_khz = source_freq_hz / 1000;
    let spiclk_apb_n = source_freq_hz / eff_clk;

    // Change from esp-idf: `input_delay_ns` also represents the GPIO matrix delay.
    let input_delay_ns = 25; // TODO: allow configuring input delay.
    let delay_apb_n = (1 + input_delay_ns) * apbclk_khz / 1000 / 1000;

    let dummy_required = delay_apb_n / spiclk_apb_n;
    let timing_miso_delay = if dummy_required > 0 {
        Some(((dummy_required + 1) * spiclk_apb_n - delay_apb_n - 1) as u8)
    } else if delay_apb_n * 4 <= spiclk_apb_n {
        None
    } else {
        Some(0)
    };

    driver
        .state
        .esp32_hack
        .extra_dummy
        .set(dummy_required as u8);
    driver
        .state
        .esp32_hack
        .timing_miso_delay
        .set(timing_miso_delay);
}

pub(super) fn set_data_mode(driver: &Driver, data_mode: Mode) {
    driver.regs().pin().modify(|_, w| {
        w.ck_idle_edge()
            .bit(matches!(data_mode, Mode::_2 | Mode::_3))
    });
    driver.regs().user().modify(|_, w| {
        w.ck_out_edge()
            .bit(matches!(data_mode, Mode::_1 | Mode::_2))
    });
}

pub(super) fn setup_full_duplex(driver: &Driver) {
    // For full-duplex, we don't need compensation according to esp-idf.
    driver.regs().ctrl2().modify(|_, w| unsafe {
        w.miso_delay_mode().bits(0);
        w.miso_delay_num().bits(0)
    });
}

pub(super) fn prepare_half_duplex(driver: &Driver, is_write: bool, dummy: u8) -> u8 {
    let mut dummy = dummy;
    driver.regs().ctrl2().modify(|_, w| {
        let mut delay_mode = 0;
        let mut delay_num = 0;

        if !is_write {
            // Values are set up in apply_config.
            let timing_miso_delay = driver.state.esp32_hack.timing_miso_delay.get();
            let extra_dummy = driver.state.esp32_hack.extra_dummy.get();
            dummy += extra_dummy;

            if let Some(delay) = timing_miso_delay {
                delay_num = if extra_dummy > 0 { delay } else { 0 };
            } else {
                let out_edge = driver.regs().user().read().ck_out_edge().bit_is_set();
                // SPI modes 1 and 2 need delay mode 1 according to esp-idf.
                delay_mode = if out_edge { 1 } else { 2 };
            }
        }

        unsafe {
            w.miso_delay_mode().bits(delay_mode);
            w.miso_delay_num().bits(delay_num)
        }
    });
    dummy
}

pub(super) fn setup_half_duplex(_driver: &Driver) {}

pub(super) fn write_address(driver: &Driver, addr: u32) {
    driver.regs().addr().write(|w| unsafe { w.bits(addr) });
}

pub(super) fn configure_datalen(driver: &Driver, rx_len: u32, tx_len: u32) {
    let len = rx_len.max(tx_len);
    driver
        .regs()
        .mosi_dlen()
        .write(|w| unsafe { w.usr_mosi_dbitlen().bits(len) });

    driver
        .regs()
        .miso_dlen()
        .write(|w| unsafe { w.usr_miso_dbitlen().bits(len) });
}
