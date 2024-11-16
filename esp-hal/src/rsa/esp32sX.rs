use core::convert::Infallible;

use crate::rsa::{
    implement_op,
    Multi,
    Rsa,
    RsaMode,
    RsaModularExponentiation,
    RsaModularMultiplication,
    RsaMultiplication,
};

impl<'d, DM: crate::Mode> Rsa<'d, DM> {
    /// After the RSA accelerator is released from reset, the memory blocks
    /// needs to be initialized, only after that peripheral should be used.
    /// This function would return without an error if the memory is
    /// initialized.
    pub fn ready(&mut self) -> nb::Result<(), Infallible> {
        if self.rsa.clean().read().clean().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        Ok(())
    }

    /// Enables/disables rsa interrupt.
    ///
    /// When enabled rsa peripheral would generate an interrupt when a operation
    /// is finished.
    pub fn enable_disable_interrupt(&mut self, enable: bool) {
        self.rsa.int_ena().write(|w| w.int_ena().bit(enable));
    }

    fn write_mode(&mut self, mode: u32) {
        self.rsa.mode().write(|w| unsafe { w.bits(mode) });
    }

    /// Enables/disables search acceleration.
    ///
    /// When enabled it would increases the performance of modular
    /// exponentiation by discarding the exponent's bits before the most
    /// significant set bit.
    ///
    /// Note: this might decrease security.
    ///
    /// For more information refer to 20.3.4 of <https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf>.
    pub fn enable_disable_search_acceleration(&mut self, enable: bool) {
        match enable {
            true => self
                .rsa
                .search_enable()
                .write(|w| w.search_enable().set_bit()),
            false => self
                .rsa
                .search_enable()
                .write(|w| w.search_enable().clear_bit()),
        };
    }

    /// Checks if the search functionality is enabled in the RSA hardware.
    pub(super) fn is_search_enabled(&mut self) -> bool {
        self.rsa.search_enable().read().search_enable().bit_is_set()
    }

    /// Sets the search position in the RSA hardware.
    pub(super) fn write_search_position(&mut self, search_position: u32) {
        self.rsa
            .search_pos()
            .write(|w| unsafe { w.bits(search_position) });
    }

    /// Enables/disables constant time acceleration.
    ///
    /// When enabled it would increases the performance of modular
    /// exponentiation by simplifying the calculation concerning the 0 bits
    /// of the exponent. I.e. lesser the hamming weight, greater the
    /// performance.
    ///
    /// Note: this might decrease security.
    ///
    /// For more information refer to 20.3.4 of <https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf>.
    pub fn enable_disable_constant_time_acceleration(&mut self, enable: bool) {
        match enable {
            true => self
                .rsa
                .constant_time()
                .write(|w| w.constant_time().clear_bit()),
            false => self
                .rsa
                .constant_time()
                .write(|w| w.constant_time().set_bit()),
        };
    }

    /// Starts the modular exponentiation operation.
    pub(super) fn write_modexp_start(&self) {
        self.rsa
            .modexp_start()
            .write(|w| w.modexp_start().set_bit());
    }

    /// Starts the multiplication operation.
    pub(super) fn write_multi_start(&self) {
        self.rsa.mult_start().write(|w| w.mult_start().set_bit());
    }

    /// Starts the modular multiplication operation.
    pub(super) fn write_modmulti_start(&self) {
        self.rsa
            .modmult_start()
            .write(|w| w.modmult_start().set_bit());
    }

    /// Clears the RSA interrupt flag.
    pub(super) fn clear_interrupt(&mut self) {
        self.rsa.int_clr().write(|w| w.int_clr().set_bit());
    }

    /// Checks if the RSA peripheral is idle.
    pub(super) fn is_idle(&self) -> bool {
        self.rsa.idle().read().idle().bit_is_set()
    }
}

pub mod operand_sizes {
    //! Marker types for the operand sizes
    use paste::paste;

    use super::{implement_op, Multi, RsaMode};

    implement_op!(
        (32, multi),
        (64, multi),
        (96, multi),
        (128, multi),
        (160, multi),
        (192, multi),
        (224, multi),
        (256, multi),
        (288, multi),
        (320, multi),
        (352, multi),
        (384, multi),
        (416, multi),
        (448, multi),
        (480, multi),
        (512, multi),
        (544, multi),
        (576, multi),
        (608, multi),
        (640, multi),
        (672, multi),
        (704, multi),
        (736, multi),
        (768, multi),
        (800, multi),
        (832, multi),
        (864, multi),
        (896, multi),
        (928, multi),
        (960, multi),
        (992, multi),
        (1024, multi),
        (1056, multi),
        (1088, multi),
        (1120, multi),
        (1152, multi),
        (1184, multi),
        (1216, multi),
        (1248, multi),
        (1280, multi),
        (1312, multi),
        (1344, multi),
        (1376, multi),
        (1408, multi),
        (1440, multi),
        (1472, multi),
        (1504, multi),
        (1536, multi),
        (1568, multi),
        (1600, multi),
        (1632, multi),
        (1664, multi),
        (1696, multi),
        (1728, multi),
        (1760, multi),
        (1792, multi),
        (1824, multi),
        (1856, multi),
        (1888, multi),
        (1920, multi),
        (1952, multi),
        (1984, multi),
        (2016, multi),
        (2048, multi)
    );

    implement_op!(
        (2080),
        (2112),
        (2144),
        (2176),
        (2208),
        (2240),
        (2272),
        (2304),
        (2336),
        (2368),
        (2400),
        (2432),
        (2464),
        (2496),
        (2528),
        (2560),
        (2592),
        (2624),
        (2656),
        (2688),
        (2720),
        (2752),
        (2784),
        (2816),
        (2848),
        (2880),
        (2912),
        (2944),
        (2976),
        (3008),
        (3040),
        (3072),
        (3104),
        (3136),
        (3168),
        (3200),
        (3232),
        (3264),
        (3296),
        (3328),
        (3360),
        (3392),
        (3424),
        (3456),
        (3488),
        (3520),
        (3552),
        (3584),
        (3616),
        (3648),
        (3680),
        (3712),
        (3744),
        (3776),
        (3808),
        (3840),
        (3872),
        (3904),
        (3936),
        (3968),
        (4000),
        (4032),
        (4064),
        (4096)
    );
}

impl<'a, 'd, T: RsaMode, DM: crate::Mode, const N: usize> RsaModularExponentiation<'a, 'd, T, DM>
where
    T: RsaMode<InputType = [u32; N]>,
{
    pub(super) fn find_search_pos(exponent: &T::InputType) -> u32 {
        for (i, byte) in exponent.iter().rev().enumerate() {
            if *byte == 0 {
                continue;
            }
            return (exponent.len() * 32) as u32 - (byte.leading_zeros() + i as u32 * 32) - 1;
        }
        0
    }

    /// Sets the modular exponentiation mode for the RSA hardware.
    pub(super) fn write_mode(rsa: &mut Rsa<'d, DM>) {
        rsa.write_mode((N - 1) as u32)
    }
}

impl<'a, 'd, T: RsaMode, DM: crate::Mode, const N: usize> RsaModularMultiplication<'a, 'd, T, DM>
where
    T: RsaMode<InputType = [u32; N]>,
{
    pub(super) fn write_mode(rsa: &mut Rsa<'d, DM>) {
        rsa.write_mode((N - 1) as u32)
    }

    pub(super) fn set_up_modular_multiplication(&mut self, operand_b: &T::InputType) {
        self.rsa.write_operand_b(operand_b);
    }
}

impl<'a, 'd, T: RsaMode + Multi, DM: crate::Mode, const N: usize> RsaMultiplication<'a, 'd, T, DM>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// Sets the multiplication mode for the RSA hardware.
    pub(super) fn write_mode(rsa: &mut Rsa<'d, DM>) {
        rsa.write_mode((N * 2 - 1) as u32)
    }

    pub(super) fn set_up_multiplication(&mut self, operand_b: &T::InputType) {
        self.rsa.write_multi_operand_b(operand_b);
    }
}
