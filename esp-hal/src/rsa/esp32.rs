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
    /// After the RSA Accelerator is released from reset, the memory blocks
    /// needs to be initialized, only after that peripheral should be used.
    /// This function would return without an error if the memory is initialized
    pub fn ready(&mut self) -> nb::Result<(), Infallible> {
        if self.rsa.clean().read().clean().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        Ok(())
    }

    /// Writes the multi-mode configuration to the RSA hardware.
    pub(super) fn write_multi_mode(&mut self, mode: u32) {
        self.rsa.mult_mode().write(|w| unsafe { w.bits(mode) });
    }

    /// Writes the modular exponentiation mode configuration to the RSA
    /// hardware.
    pub(super) fn write_modexp_mode(&mut self, mode: u32) {
        self.rsa.modexp_mode().write(|w| unsafe { w.bits(mode) });
    }

    /// Starts the modular exponentiation operation.
    pub(super) fn write_modexp_start(&mut self) {
        self.rsa
            .modexp_start()
            .write(|w| w.modexp_start().set_bit());
    }

    /// Starts the multiplication operation.
    pub(super) fn write_multi_start(&mut self) {
        self.rsa.mult_start().write(|w| w.mult_start().set_bit());
    }

    /// Clears the RSA interrupt flag.
    pub(super) fn clear_interrupt(&mut self) {
        self.rsa.interrupt().write(|w| w.interrupt().set_bit());
    }

    /// Checks if the RSA peripheral is idle.
    pub(super) fn is_idle(&mut self) -> bool {
        self.rsa.interrupt().read().bits() == 1
    }
}

/// Module defining marker types for various RSA operand sizes.
pub mod operand_sizes {
    //! Marker types for the operand sizes
    use paste::paste;

    use super::{implement_op, Multi, RsaMode};

    implement_op!(
        (512, multi),
        (1024, multi),
        (1536, multi),
        (2048, multi),
        (2560),
        (3072),
        (3584),
        (4096)
    );
}

impl<'a, 'd, T: RsaMode, DM: crate::Mode, const N: usize> RsaModularMultiplication<'a, 'd, T, DM>
where
    T: RsaMode<InputType = [u32; N]>,
{
    pub(super) fn write_mode(rsa: &mut Rsa<'d, DM>) {
        rsa.write_multi_mode((N / 16 - 1) as u32)
    }

    /// Starts the modular multiplication operation.
    ///
    /// For more information refer to 24.3.2 of <https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf>.
    pub fn start_modular_multiplication(&mut self, operand_b: &T::InputType) {
        self.rsa.write_multi_start();
        self.rsa.wait_for_idle();

        self.rsa.write_operand_a(operand_b);
        self.rsa.write_multi_start();
    }
}

impl<'a, 'd, T: RsaMode, DM: crate::Mode, const N: usize> RsaModularExponentiation<'a, 'd, T, DM>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// Sets the modular exponentiation mode for the RSA hardware.
    pub(super) fn write_mode(rsa: &mut Rsa<'d, DM>) {
        rsa.write_modexp_mode((N / 16 - 1) as u32)
    }
}

impl<'a, 'd, T: RsaMode + Multi, DM: crate::Mode, const N: usize> RsaMultiplication<'a, 'd, T, DM>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// Starts the multiplication operation.
    pub fn start_multiplication(&mut self, operand_b: &T::InputType) {
        self.rsa.write_multi_operand_b(operand_b);
        self.start();
    }

    /// Sets the multiplication mode for the RSA hardware.
    pub(super) fn write_mode(rsa: &mut Rsa<'d, DM>) {
        rsa.write_multi_mode(((N * 2) / 16 + 7) as u32)
    }

    /// Starts the multiplication operation on the RSA hardware.
    pub(super) fn start(&mut self) {
        self.rsa.write_multi_start();
    }
}
