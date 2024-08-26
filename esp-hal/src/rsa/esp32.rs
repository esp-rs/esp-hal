use core::{
    convert::Infallible,
    marker::PhantomData,
    ptr::{copy_nonoverlapping, write_bytes},
};

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

    unsafe fn write_multi_operand_a<const N: usize>(&mut self, operand_a: &[u32; N]) {
        copy_nonoverlapping(operand_a.as_ptr(), self.rsa.x_mem(0).as_ptr(), N);
        write_bytes(self.rsa.x_mem(0).as_ptr().add(N), 0, N);
    }

    unsafe fn write_multi_operand_b<const N: usize>(&mut self, operand_b: &[u32; N]) {
        write_bytes(self.rsa.z_mem(0).as_ptr(), 0, N);
        copy_nonoverlapping(operand_b.as_ptr(), self.rsa.z_mem(0).as_ptr().add(N), N);
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
    /// Creates an instance of `RsaMultiplication`.
    ///
    /// `m_prime` can be calculated using `-(modular multiplicative inverse of
    /// modulus) mod 2^32`.
    ///
    /// For more information refer to 24.3.2 of <https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf>.
    pub fn new(rsa: &'a mut Rsa<'d, DM>, modulus: &T::InputType, m_prime: u32) -> Self {
        Self::set_mode(rsa);
        unsafe {
            rsa.write_modulus(modulus);
        }
        rsa.write_mprime(m_prime);

        Self {
            rsa,
            phantom: PhantomData,
        }
    }

    fn set_mode(rsa: &mut Rsa<'d, DM>) {
        rsa.write_multi_mode((N / 16 - 1) as u32)
    }

    /// Starts the first step of modular multiplication operation.
    ///
    /// `r` can be calculated using `2 ^ ( bitlength * 2 ) mod modulus`.
    ///
    /// For more information refer to 24.3.2 of <https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf>.
    pub fn start_step1(&mut self, operand_a: &T::InputType, r: &T::InputType) {
        unsafe {
            self.rsa.write_operand_a(operand_a);
            self.rsa.write_r(r);
        }
        self.start();
    }

    /// Starts the second step of modular multiplication operation.
    ///
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully. `start_step1` must be called
    /// before calling this function.
    pub fn start_step2(&mut self, operand_b: &T::InputType) {
        while !self.rsa.is_idle() {}

        self.rsa.clear_interrupt();
        unsafe {
            self.rsa.write_operand_a(operand_b);
        }
        self.start();
    }

    fn start(&mut self) {
        self.rsa.write_multi_start();
    }
}

impl<'a, 'd, T: RsaMode, DM: crate::Mode, const N: usize> RsaModularExponentiation<'a, 'd, T, DM>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// Creates an instance of `RsaModularExponentiation`.
    ///
    /// `m_prime` can be calculated using `-(modular multiplicative inverse of
    /// modulus) mod 2^32`.
    ///
    /// For more information refer to 24.3.2 of <https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf>.
    pub fn new(
        rsa: &'a mut Rsa<'d, DM>,
        exponent: &T::InputType,
        modulus: &T::InputType,
        m_prime: u32,
    ) -> Self {
        Self::set_mode(rsa);
        unsafe {
            rsa.write_operand_b(exponent);
            rsa.write_modulus(modulus);
        }
        rsa.write_mprime(m_prime);
        Self {
            rsa,
            phantom: PhantomData,
        }
    }

    /// Sets the modular exponentiation mode for the RSA hardware.
    pub(super) fn set_mode(rsa: &mut Rsa<'d, DM>) {
        rsa.write_modexp_mode((N / 16 - 1) as u32)
    }

    /// Starts the modular exponentiation operation on the RSA hardware.
    pub(super) fn start(&mut self) {
        self.rsa.write_modexp_start();
    }
}

impl<'a, 'd, T: RsaMode + Multi, DM: crate::Mode, const N: usize> RsaMultiplication<'a, 'd, T, DM>
where
    T: RsaMode<InputType = [u32; N]>,
{
    /// Creates an instance of `RsaMultiplication`.
    pub fn new(rsa: &'a mut Rsa<'d, DM>) -> Self {
        Self::set_mode(rsa);
        Self {
            rsa,
            phantom: PhantomData,
        }
    }

    /// Starts the multiplication operation.
    pub fn start_multiplication(&mut self, operand_a: &T::InputType, operand_b: &T::InputType) {
        unsafe {
            self.rsa.write_multi_operand_a(operand_a);
            self.rsa.write_multi_operand_b(operand_b);
        }
        self.start();
    }

    /// Sets the multiplication mode for the RSA hardware.
    pub(super) fn set_mode(rsa: &mut Rsa<'d, DM>) {
        rsa.write_multi_mode(((N * 2) / 16 + 7) as u32)
    }

    /// Starts the multiplication operation on the RSA hardware.
    pub(super) fn start(&mut self) {
        self.rsa.write_multi_start();
    }
}
