//! CORDIC tests. No external wiring is required.

//% CHIP_FILTER: cordic_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    cordic::{ClockSource, Config, ConfigError, Cordic, Error, Precision},
    peripherals::CORDIC,
    time::Rate,
};
use hil_test as _;

const Q15_TOLERANCE: u16 = 2;
const Q31_TOLERANCE: u32 = 512;
const FRACTIONAL_XTAL_RATE: Rate = Rate::from_mhz(24);
const ATAN_INPUT_Q31: i32 = 0x4000_0000;
const ATAN_EXPECTED_Q31: i32 = 316_933_406;
const SIN_COS_PI_OVER_8_Q31: (i32, i32, i32) = (0x1000_0000, 821_806_413, 1_984_016_189);

fn check_q15(actual: i16, expected: i16) {
    assert!(
        actual.abs_diff(expected) <= Q15_TOLERANCE,
        "{actual} != {expected}"
    );
}

fn check_q31(actual: i32, expected: i32) {
    assert!(
        actual.abs_diff(expected) <= Q31_TOLERANCE,
        "{actual} != {expected}"
    );
}

fn check_atan_q31(cordic: &mut Cordic<'_>) {
    check_q31(cordic.atan_q31(ATAN_INPUT_Q31).unwrap(), ATAN_EXPECTED_Q31);
}

fn check_hyperbolic_vectoring<'d>(
    cordic: &mut Cordic<'d>,
    opcode: u8,
    calculate_q15: fn(&mut Cordic<'d>, i16) -> Result<i16, Error>,
    calculate_q31: fn(&mut Cordic<'d>, i32) -> Result<i32, Error>,
) {
    let csr = CORDIC::regs().csr_cfg();
    check_q31(calculate_q31(cordic, -865_435_910).unwrap(), -1_197_765_218);
    assert_eq!(csr.read().func().bits(), opcode, "Q31");

    check_q15(calculate_q15(cordic, -13_205).unwrap(), -18_275);
    assert_eq!(csr.read().func().bits(), opcode, "Q15");

    assert_eq!(calculate_q15(cordic, -13_206), Err(Error::InvalidInput));
    assert_eq!(calculate_q31(cordic, 865_435_911), Err(Error::InvalidInput));
}

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[test]
    fn trigonometry() {
        let p = esp_hal::init(esp_hal::Config::default());
        let mut cordic = Cordic::new(p.CORDIC, Config::default()).unwrap();
        check_atan_q31(&mut cordic);
        check_q15(cordic.atan_q15(-0x4000).unwrap(), -4_836);

        for (angle, sin, cos) in [
            (0, 0, i32::MAX),
            SIN_COS_PI_OVER_8_Q31,
            (-0x6000_0000, -1_518_500_250, -1_518_500_250),
            (i32::MIN, 0, i32::MIN),
        ] {
            let (actual_sin, actual_cos) = cordic.sin_cos_q31(angle).unwrap();
            check_q31(actual_sin, sin);
            check_q31(actual_cos, cos);

            let (actual_sin, actual_cos) = cordic.sin_cos_q15((angle >> 16) as i16).unwrap();
            check_q15(actual_sin, (sin >> 16) as i16);
            check_q15(actual_cos, (cos >> 16) as i16);
        }

        let csr = CORDIC::regs().csr_cfg();
        let (angle, sin, cos) = SIN_COS_PI_OVER_8_Q31;
        let angle_q15 = (angle >> 16) as i16;
        check_q31(cordic.sin_q31(-angle).unwrap(), -sin);
        assert_eq!(csr.read().func().bits(), 1);
        assert!(csr.read().res_num().bit_is_clear());
        check_q15(cordic.sin_q15(angle_q15).unwrap(), (sin >> 16) as i16);
        check_q31(cordic.cos_q31(angle).unwrap(), cos);
        assert_eq!(csr.read().func().bits(), 0);
        check_q15(cordic.cos_q15(angle_q15).unwrap(), (cos >> 16) as i16);

        let (x, y) = (-0x2000_0000, -0x4000_0000);
        let (x_q15, y_q15) = ((x >> 16) as i16, (y >> 16) as i16);
        let (phase, modulus) = cordic.phase_modulus_q31(x, y).unwrap();
        check_q31(phase, -1_390_675_230);
        check_q31(modulus, 1_200_479_854);
        check_q31(cordic.phase_q31(x, y).unwrap(), phase);
        assert_eq!(csr.read().func().bits(), 2);
        assert!(csr.read().res_num().bit_is_clear());

        let (phase, modulus) = cordic.phase_modulus_q15(x_q15, y_q15).unwrap();
        check_q15(phase, -21_220);
        check_q15(modulus, 18_318);
        check_q15(cordic.phase_q15(x_q15, y_q15).unwrap(), phase);

        check_q31(cordic.modulus_q31(x, y).unwrap(), 1_200_479_854);
        assert_eq!(csr.read().func().bits(), 3);
        check_q15(cordic.modulus_q15(x_q15, y_q15).unwrap(), 18_318);

        assert_eq!(cordic.modulus_q31(0, 0), Ok(0));
        assert_eq!(cordic.modulus_q15(0, 0), Ok(0));
        assert!(csr.read().arg_num().bit_is_set());
        assert_eq!(cordic.phase_q31(0, 0), Err(Error::InvalidInput));
        assert_eq!(cordic.phase_modulus_q15(0, 0), Err(Error::InvalidInput));

        assert_eq!(
            cordic.phase_modulus_q31(i32::MIN, 0),
            Err(Error::InvalidInput)
        );
        assert_eq!(
            cordic.phase_modulus_q15(0x6000, 0x6000),
            Err(Error::InvalidInput)
        );
        assert_eq!(
            cordic.modulus_q31(0x6000_0000, 0x6000_0000),
            Err(Error::InvalidInput)
        );
    }

    #[test]
    fn clock_configuration() {
        let mut p = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );
        let default = Config::default();
        assert_eq!(
            Cordic::new(
                p.CORDIC.reborrow(),
                default.with_frequency(Rate::from_hz(0))
            )
            .unwrap_err(),
            ConfigError::UnreachableClockRate
        );
        let mut cordic = Cordic::new(p.CORDIC.reborrow(), default).unwrap();

        let regs = p.HP_SYS_CLKRST.register_block();
        let control = regs.cordic_ctrl0();
        let divider = regs.cordic_ctrl1();
        assert_eq!(cordic.frequency(), Rate::from_mhz(160));
        assert_eq!(control.read().clk_src_sel().bits(), 2);
        check_atan_q31(&mut cordic);

        let xtal = default
            .with_clock_source(ClockSource::Xtal)
            .with_frequency(FRACTIONAL_XTAL_RATE);
        cordic.apply_config(&xtal).unwrap();
        assert_eq!(cordic.frequency(), FRACTIONAL_XTAL_RATE);
        assert_eq!(control.read().clk_src_sel().bits(), 0);
        let div = divider.read();
        assert_eq!(div.clk_div_num().bits(), 0);
        assert_eq!(div.clk_div_numerator().bits(), 2);
        assert_eq!(div.clk_div_denonimator().bits(), 3);
        check_atan_q31(&mut cordic);

        let rc_fast = default.with_clock_source(ClockSource::RcFast);
        cordic.apply_config(&rc_fast).unwrap();
        let slow_target = Rate::from_hz(cordic.frequency().as_hz().div_ceil(256));
        cordic
            .apply_config(&rc_fast.with_frequency(slow_target))
            .unwrap();
        let slow_rate = cordic.frequency();
        let slow_divider = divider.read();
        assert!(slow_rate.as_hz().abs_diff(slow_target.as_hz()) <= 1);
        assert_eq!(control.read().clk_src_sel().bits(), 1);
        assert!(slow_divider.clk_div_num().bits() >= 254);

        let invalid = xtal
            .with_frequency(Rate::from_mhz(41))
            .with_precision(Precision::Iterations4);
        assert_eq!(
            cordic.apply_config(&invalid),
            Err(ConfigError::UnreachableClockRate)
        );
        assert_eq!(cordic.frequency(), slow_rate);
        assert_eq!(control.read().clk_src_sel().bits(), 1);
        assert_eq!(divider.read().bits(), slow_divider.bits());
        check_atan_q31(&mut cordic);
        assert_eq!(CORDIC::regs().csr_cfg().read().pres().bits(), 5);

        drop(cordic);
        let ctrl = control.read();
        assert!(ctrl.clk_en().bit_is_clear());
        assert!(ctrl.apb_clk_en().bit_is_clear());
        assert!(ctrl.sys_clk_en().bit_is_clear());

        let mut cordic = Cordic::new(p.CORDIC, xtal).unwrap();
        assert_eq!(cordic.frequency(), FRACTIONAL_XTAL_RATE);
        assert_eq!(control.read().clk_src_sel().bits(), 0);
        check_atan_q31(&mut cordic);
    }

    #[test]
    fn calculation_precision() {
        let p = esp_hal::init(esp_hal::Config::default());
        let config = Config::default()
            .with_clock_source(ClockSource::Xtal)
            .with_frequency(FRACTIONAL_XTAL_RATE);
        let mut cordic =
            Cordic::new(p.CORDIC, config.with_precision(Precision::Iterations4)).unwrap();
        let divider = p.HP_SYS_CLKRST.register_block().cordic_ctrl1();
        let original_divider = divider.read().bits();
        let csr = CORDIC::regs().csr_cfg();
        let (angle, expected_sin, expected_cos) = SIN_COS_PI_OVER_8_Q31;

        let coarse = cordic.sin_cos_q31(angle).unwrap();
        assert_eq!(csr.read().pres().bits(), 0);

        cordic.apply_config(&config).unwrap();
        assert_eq!(cordic.frequency(), FRACTIONAL_XTAL_RATE);
        assert_eq!(divider.read().bits(), original_divider);
        let (sin, cos) = cordic.sin_cos_q31(angle).unwrap();
        assert_eq!(csr.read().pres().bits(), 5);
        assert_ne!((sin, cos), coarse);
        check_q31(sin, expected_sin);
        check_q31(cos, expected_cos);
    }

    #[test]
    fn timeout_recovery() {
        let p = esp_hal::init(esp_hal::Config::default());
        let config = Config::default()
            .with_clock_source(ClockSource::Xtal)
            .with_frequency(FRACTIONAL_XTAL_RATE)
            .with_precision(Precision::Iterations16);
        let mut cordic = Cordic::new(p.CORDIC, config).unwrap();
        let csr = CORDIC::regs().csr_cfg();
        let before_timeout = cordic.atan_q31(ATAN_INPUT_Q31).unwrap();

        let clocks = p.HP_SYS_CLKRST.register_block();
        let control = clocks.cordic_ctrl0();
        let divider = clocks.cordic_ctrl1();
        let original_control = control.read();
        let original_divider = divider.read().bits();
        // Stop the computation clock while leaving the APB registers accessible.
        control.modify(|_, w| w.clk_en().clear_bit());
        let result = cordic.atan_q31(-ATAN_INPUT_Q31);
        let reset_func = csr.read().func().bits();
        control.modify(|_, w| w.clk_en().set_bit());
        assert_eq!(result, Err(Error::Timeout));
        assert_eq!(reset_func, 0);
        assert_eq!(control.read().bits(), original_control.bits());
        assert_eq!(divider.read().bits(), original_divider);
        check_q31(cordic.atan_q31(-ATAN_INPUT_Q31).unwrap(), -before_timeout);
        assert_eq!(csr.read().pres().bits(), 3);
    }

    #[test]
    fn square_root() {
        let p = esp_hal::init(esp_hal::Config::default());
        let default = Config::default();
        let mut cordic = Cordic::new(p.CORDIC, default).unwrap();

        assert_eq!(cordic.sqrt_q31(-1), Err(Error::InvalidInput));
        assert_eq!(cordic.sqrt_q15(0), Ok(0));

        check_q15(cordic.sqrt_q15(1).unwrap(), 181);
        assert!(cordic.sqrt_q31(1).unwrap().abs_diff(46_341) <= 1);
        check_q31(cordic.sqrt_q31(0x0200_0000).unwrap(), 0x1000_0000);
        let csr = CORDIC::regs().csr_cfg();
        assert_eq!(csr.read().pres().bits(), 2);

        cordic
            .apply_config(&default.with_precision(Precision::Iterations16))
            .unwrap();
        assert_eq!(cordic.sqrt_q15(0), Err(Error::UnsupportedPrecision));
        assert_eq!(cordic.sqrt_q31(1), Err(Error::UnsupportedPrecision));

        cordic
            .apply_config(&default.with_precision(Precision::Iterations12))
            .unwrap();
        check_q15(cordic.sqrt_q15(0x4000).unwrap(), 23_170);
        check_q31(cordic.sqrt_q31(0x4000_0000).unwrap(), 1_518_500_250);
        assert_eq!(csr.read().pres().bits(), 2);
    }

    #[test]
    fn hyperbolic() {
        const ATANH_FUNC: u8 = 7;

        let p = esp_hal::init(esp_hal::Config::default());
        let mut cordic = Cordic::new(p.CORDIC, Config::default()).unwrap();

        let input_q15 = -18_317;
        let input_q31 = 1_200_443_359;

        let (sinh, cosh) = cordic.sinh_cosh_q15(input_q15).unwrap();
        check_q15(sinh, -22_378);
        check_q15(cosh, 27_735);
        assert_eq!(
            cordic.sinh_cosh_q15(input_q15 - 1),
            Err(Error::InvalidInput)
        );
        check_q15(cordic.sinh_q15(input_q15).unwrap(), sinh);

        let (sinh, cosh) = cordic.sinh_cosh_q31(input_q31).unwrap();
        check_q31(sinh, 1_466_622_676);
        check_q31(cosh, 1_817_664_319);

        let csr = CORDIC::regs().csr_cfg();
        check_q31(cordic.sinh_q31(input_q31).unwrap(), sinh);
        assert_eq!(csr.read().func().bits(), 6);
        assert!(csr.read().res_num().bit_is_clear());

        check_q31(cordic.cosh_q31(input_q31).unwrap(), 1_817_664_319);
        assert_eq!(csr.read().func().bits(), 5);
        assert_eq!(cordic.cosh_q31(input_q31 + 1), Err(Error::InvalidInput));
        check_q15(cordic.cosh_q15(input_q15).unwrap(), 27_735);

        check_hyperbolic_vectoring(
            &mut cordic,
            ATANH_FUNC,
            Cordic::atanh_q15,
            Cordic::atanh_q31,
        );
        let (sinh, cosh) = cordic.sinh_cosh_q15(0).unwrap();
        check_q15(sinh, 0);
        check_q15(cosh, 16_384);
        check_atan_q31(&mut cordic);
    }

    #[test]
    fn natural_logarithm() {
        const LN_FUNC: u8 = 8;

        let p = esp_hal::init(esp_hal::Config::default());
        let mut cordic = Cordic::new(p.CORDIC, Config::default()).unwrap();

        // The logarithm's input ratio gives ln(x) / 4 = atanh(2 * ratio) / 2.
        check_hyperbolic_vectoring(
            &mut cordic,
            LN_FUNC,
            Cordic::ln_from_ratio_q15,
            Cordic::ln_from_ratio_q31,
        );
    }
}
