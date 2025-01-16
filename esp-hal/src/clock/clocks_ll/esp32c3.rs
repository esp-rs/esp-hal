use crate::{
    clock::{ApbClock, Clock, CpuClock, PllClock, XtalClock},
    rom::{regi2c_write, regi2c_write_mask},
};

const I2C_BBPLL: u32 = 0x66;
const I2C_BBPLL_HOSTID: u32 = 0;

const I2C_BBPLL_MODE_HF: u32 = 4;

const I2C_BBPLL_OC_REF_DIV: u32 = 2;
const I2C_BBPLL_OC_DCHGP_LSB: u32 = 4;
const I2C_BBPLL_OC_DIV_7_0: u32 = 3;

const I2C_BBPLL_OC_DR1: u32 = 5;
const I2C_BBPLL_OC_DR1_MSB: u32 = 2;
const I2C_BBPLL_OC_DR1_LSB: u32 = 0;

const I2C_BBPLL_OC_DR3: u32 = 5;
const I2C_BBPLL_OC_DR3_MSB: u32 = 6;
const I2C_BBPLL_OC_DR3_LSB: u32 = 4;

const I2C_BBPLL_OC_DCUR: u32 = 6;

const I2C_BBPLL_OC_VCO_DBIAS: u32 = 9;
const I2C_BBPLL_OC_VCO_DBIAS_MSB: u32 = 1;
const I2C_BBPLL_OC_VCO_DBIAS_LSB: u32 = 0;

const I2C_BBPLL_OC_DHREF_SEL: u32 = 6;
const I2C_BBPLL_OC_DHREF_SEL_MSB: u32 = 5;
const I2C_BBPLL_OC_DHREF_SEL_LSB: u32 = 4;

const I2C_BBPLL_OC_DLREF_SEL: u32 = 6;
const I2C_BBPLL_OC_DLREF_SEL_MSB: u32 = 7;
const I2C_BBPLL_OC_DLREF_SEL_LSB: u32 = 6;

const I2C_MST_ANA_CONF0_REG: u32 = 0x6000_e040;
const I2C_MST_BBPLL_STOP_FORCE_HIGH: u32 = 1 << 3;
const I2C_MST_BBPLL_STOP_FORCE_LOW: u32 = 1 << 2;

pub(crate) fn esp32c3_rtc_bbpll_configure(xtal_freq: XtalClock, pll_freq: PllClock) {
    let system = crate::peripherals::SYSTEM::regs();

    let div_ref: u32;
    let div7_0: u32;
    let dr1: u32;
    let dr3: u32;
    let dchgp: u32;
    let dcur: u32;
    let dbias: u32;

    unsafe {
        let clear_reg_mask = |reg, mask: u32| {
            (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !mask)
        };
        let set_reg_mask = |reg, mask: u32| {
            (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | mask)
        };

        clear_reg_mask(I2C_MST_ANA_CONF0_REG, I2C_MST_BBPLL_STOP_FORCE_HIGH);
        set_reg_mask(I2C_MST_ANA_CONF0_REG, I2C_MST_BBPLL_STOP_FORCE_LOW);
    }

    if matches!(pll_freq, PllClock::Pll480MHz) {
        // Set this register to let the digital part know 480M PLL is used
        system
            .cpu_per_conf()
            .modify(|_, w| w.pll_freq_sel().set_bit());

        // Configure 480M PLL
        match xtal_freq {
            XtalClock::_40M => {
                div_ref = 0;
                div7_0 = 8;
                dr1 = 0;
                dr3 = 0;
                dchgp = 5;
                dcur = 3;
                dbias = 2;
            }

            XtalClock::_32M => {
                div_ref = 1;
                div7_0 = 26;
                dr1 = 1;
                dr3 = 1;
                dchgp = 4;
                dcur = 0;
                dbias = 2;
            }

            XtalClock::Other(_) => {
                div_ref = 0;
                div7_0 = 8;
                dr1 = 0;
                dr3 = 0;
                dchgp = 5;
                dcur = 3;
                dbias = 2;
            }
        }

        regi2c_write!(I2C_BBPLL, I2C_BBPLL_MODE_HF, 0x6b);
    } else {
        // Clear this register to let the digital part know 320M PLL is used
        system
            .cpu_per_conf()
            .modify(|_, w| w.pll_freq_sel().clear_bit());

        // Configure 320M PLL
        match xtal_freq {
            XtalClock::_40M => {
                div_ref = 0;
                div7_0 = 4;
                dr1 = 0;
                dr3 = 0;
                dchgp = 5;
                dcur = 3;
                dbias = 2;
            }

            XtalClock::_32M => {
                div_ref = 1;
                div7_0 = 6;
                dr1 = 0;
                dr3 = 0;
                dchgp = 5;
                dcur = 3;
                dbias = 2;
            }

            XtalClock::Other(_) => {
                div_ref = 0;
                div7_0 = 4;
                dr1 = 0;
                dr3 = 0;
                dchgp = 5;
                dcur = 3;
                dbias = 2;
            }
        }

        regi2c_write!(I2C_BBPLL, I2C_BBPLL_MODE_HF, 0x69);
    }

    let i2c_bbpll_lref = (dchgp << I2C_BBPLL_OC_DCHGP_LSB) | div_ref;
    let i2c_bbpll_div_7_0 = div7_0;
    let i2c_bbpll_dcur =
        (2 << I2C_BBPLL_OC_DLREF_SEL_LSB) | (1 << I2C_BBPLL_OC_DHREF_SEL_LSB) | dcur;

    regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_REF_DIV, i2c_bbpll_lref);

    regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_DIV_7_0, i2c_bbpll_div_7_0);

    regi2c_write_mask!(I2C_BBPLL, I2C_BBPLL_OC_DR1, dr1);

    regi2c_write_mask!(I2C_BBPLL, I2C_BBPLL_OC_DR3, dr3);

    regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_DCUR, i2c_bbpll_dcur);

    regi2c_write_mask!(I2C_BBPLL, I2C_BBPLL_OC_VCO_DBIAS, dbias);

    regi2c_write_mask!(I2C_BBPLL, I2C_BBPLL_OC_DHREF_SEL, 2);

    regi2c_write_mask!(I2C_BBPLL, I2C_BBPLL_OC_DLREF_SEL, 1);
}

pub(crate) fn esp32c3_rtc_bbpll_enable() {
    let rtc_cntl = crate::peripherals::LPWR::regs();

    rtc_cntl.options0().modify(|_, w| {
        w.bb_i2c_force_pd()
            .clear_bit()
            .bbpll_force_pd()
            .clear_bit()
            .bbpll_i2c_force_pd()
            .clear_bit()
    });
}

pub(crate) fn esp32c3_rtc_update_to_xtal(freq: XtalClock, _div: u32) {
    crate::rom::ets_update_cpu_frequency_rom(freq.mhz());

    let system_control = crate::peripherals::SYSTEM::regs();
    unsafe {
        // Set divider from XTAL to APB clock. Need to set divider to 1 (reg. value 0)
        // first.
        system_control.sysclk_conf().modify(|_, w| {
            w.pre_div_cnt()
                .bits(0)
                .pre_div_cnt()
                .bits((_div - 1) as u16)
        });

        // No need to adjust the REF_TICK

        // Switch clock source
        system_control
            .sysclk_conf()
            .modify(|_, w| w.soc_clk_sel().bits(0));
    }
}

pub(crate) fn esp32c3_rtc_freq_to_pll_mhz(cpu_clock_speed: CpuClock) {
    let system_control = crate::peripherals::SYSTEM::regs();

    unsafe {
        system_control
            .sysclk_conf()
            .modify(|_, w| w.pre_div_cnt().bits(0).soc_clk_sel().bits(1));
        system_control.cpu_per_conf().modify(|_, w| {
            w.cpuperiod_sel().bits(match cpu_clock_speed {
                CpuClock::_80MHz => 0,
                CpuClock::_160MHz => 1,
            })
        });
    }

    crate::rom::ets_update_cpu_frequency_rom(cpu_clock_speed.mhz());
}

pub(crate) fn esp32c3_rtc_apb_freq_update(apb_freq: ApbClock) {
    let rtc_cntl = crate::peripherals::LPWR::regs();
    let value = ((apb_freq.hz() >> 12) & u16::MAX as u32)
        | (((apb_freq.hz() >> 12) & u16::MAX as u32) << 16);

    rtc_cntl
        .store5()
        .modify(|_, w| unsafe { w.scratch5().bits(value) });
}
