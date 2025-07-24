use core::convert::Infallible;

use crate::rsa::{
    Multi,
    Rsa,
    RsaMode,
    RsaModularExponentiation,
    RsaModularMultiplication,
    RsaMultiplication,
};

impl<Dm: crate::DriverMode> Rsa<'_, Dm> {
    /// After the RSA accelerator is released from reset, the memory blocks
    /// needs to be initialized, only after that peripheral should be used.
    /// This function would return without an error if the memory is
    /// initialized.
    pub fn ready(&mut self) -> nb::Result<(), Infallible> {
        if self.regs().clean().read().clean().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        Ok(())
    }

    /// Enables/disables rsa interrupt.
    ///
    /// When enabled rsa peripheral would generate an interrupt when a operation
    /// is finished.
    pub fn enable_disable_interrupt(&mut self, enable: bool) {
        self.regs().int_ena().write(|w| w.int_ena().bit(enable));
    }

    fn write_mode(&mut self, mode: u32) {
        self.regs().mode().write(|w| unsafe { w.bits(mode) });
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
        self.regs()
            .search_enable()
            .write(|w| w.search_enable().bit(enable));
    }

    /// Checks if the search functionality is enabled in the RSA hardware.
    pub(super) fn is_search_enabled(&mut self) -> bool {
        self.regs()
            .search_enable()
            .read()
            .search_enable()
            .bit_is_set()
    }

    /// Sets the search position in the RSA hardware.
    pub(super) fn write_search_position(&mut self, search_position: u32) {
        self.regs()
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
        self.regs()
            .constant_time()
            .write(|w| w.constant_time().bit(!enable));
    }

    /// Starts the modular exponentiation operation.
    pub(super) fn write_modexp_start(&self) {
        self.regs()
            .modexp_start()
            .write(|w| w.modexp_start().set_bit());
    }

    /// Starts the multiplication operation.
    pub(super) fn write_multi_start(&self) {
        self.regs().mult_start().write(|w| w.mult_start().set_bit());
    }

    /// Starts the modular multiplication operation.
    pub(super) fn write_modmulti_start(&self) {
        self.regs()
            .modmult_start()
            .write(|w| w.modmult_start().set_bit());
    }

    /// Clears the RSA interrupt flag.
    pub(super) fn clear_interrupt(&mut self) {
        self.regs().int_clr().write(|w| w.int_clr().set_bit());
    }

    /// Checks if the RSA peripheral is idle.
    pub(super) fn is_idle(&self) -> bool {
        self.regs().idle().read().idle().bit_is_set()
    }
}

impl<'d, T: RsaMode, Dm: crate::DriverMode, const N: usize> RsaModularExponentiation<'_, 'd, T, Dm>
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
    pub(super) fn write_mode(rsa: &mut Rsa<'d, Dm>) {
        rsa.write_mode((N - 1) as u32)
    }
}

impl<'d, T: RsaMode, Dm: crate::DriverMode, const N: usize> RsaModularMultiplication<'_, 'd, T, Dm>
where
    T: RsaMode<InputType = [u32; N]>,
{
    pub(super) fn write_mode(rsa: &mut Rsa<'d, Dm>) {
        rsa.write_mode((N - 1) as u32)
    }

    pub(super) fn set_up_modular_multiplication(&mut self, operand_b: &T::InputType) {
        self.rsa.write_operand_b(operand_b);
    }
}

impl<'d, T: RsaMode + Multi, Dm: crate::DriverMode, const N: usize> RsaMultiplication<'_, 'd, T, Dm>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// Sets the multiplication mode for the RSA hardware.
    pub(super) fn write_mode(rsa: &mut Rsa<'d, Dm>) {
        rsa.write_mode((N * 2 - 1) as u32)
    }

    pub(super) fn set_up_multiplication(&mut self, operand_b: &T::InputType) {
        self.rsa.write_multi_operand_b(operand_b);
    }
}
