//! RSA Accelerator support.
//!
//! This module provides functions and structs for multi precision arithmetic
//! operations used in RSA asym-metric cipher algorithms
//!
//! ### Features
//! The RSA peripheral supports maximum operand of the following sizes for each
//! individual chips:
//!
//! | Feature               | ESP32| ESP32-C3| ESP32-C6| ESP32-S2| ESP32-S3|
//! |---------------------- |------|---------|---------|---------|---------|
//! |modular exponentiation |4096  |3072     |3072     |4096     |4096     |
//! |modular multiplication |4096  |3072     |3072     |4096     |4096     |
//! |multiplication         |2048  |1536     |1536     |2048     |2048     |

use core::{convert::Infallible, marker::PhantomData, ptr::copy_nonoverlapping};

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{
        generic::{Reg, RegisterSpec, Resettable, Writable},
        RSA,
    },
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

#[cfg_attr(esp32s2, path = "esp32sX.rs")]
#[cfg_attr(esp32s3, path = "esp32sX.rs")]
#[cfg_attr(esp32c3, path = "esp32cX.rs")]
#[cfg_attr(esp32c6, path = "esp32cX.rs")]
#[cfg_attr(esp32, path = "esp32.rs")]
mod rsa_spec_impl;

pub use rsa_spec_impl::operand_sizes;

/// RSA peripheral container
pub struct Rsa<'d> {
    rsa: PeripheralRef<'d, RSA>,
}

impl<'d> Rsa<'d> {
    pub fn new(
        rsa: impl Peripheral<P = RSA> + 'd,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(rsa);
        let mut ret = Self { rsa };
        ret.init(peripheral_clock_control);
        ret
    }

    fn init(&mut self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(PeripheralEnable::Rsa);
    }

    unsafe fn write_operand_b<const N: usize>(&mut self, operand_b: &[u8; N]) {
        copy_nonoverlapping(
            operand_b.as_ptr(),
            self.rsa.y_mem.as_mut_ptr() as *mut u8,
            N,
        );
    }

    unsafe fn write_modulus<const N: usize>(&mut self, modulus: &[u8; N]) {
        copy_nonoverlapping(modulus.as_ptr(), self.rsa.m_mem.as_mut_ptr() as *mut u8, N);
    }

    fn write_mprime(&mut self, m_prime: u32) {
        Self::write_to_register(&mut self.rsa.m_prime, m_prime);
    }

    unsafe fn write_operand_a<const N: usize>(&mut self, operand_a: &[u8; N]) {
        copy_nonoverlapping(
            operand_a.as_ptr(),
            self.rsa.x_mem.as_mut_ptr() as *mut u8,
            N,
        );
    }

    unsafe fn write_r<const N: usize>(&mut self, r: &[u8; N]) {
        copy_nonoverlapping(r.as_ptr(), self.rsa.z_mem.as_mut_ptr() as *mut u8, N);
    }

    unsafe fn read_out<const N: usize>(&mut self, outbuf: &mut [u8; N]) {
        copy_nonoverlapping(self.rsa.z_mem.as_ptr() as *const u8, outbuf.as_mut_ptr(), N);
    }

    fn write_to_register<T>(reg: &mut Reg<T>, data: u32)
    where
        T: RegisterSpec<Ux = u32> + Resettable + Writable,
    {
        reg.write(|w| unsafe { w.bits(data) });
    }
}

mod sealed {
    pub trait RsaMode {
        type InputType;
    }
    pub trait Multi: RsaMode {
        type OutputType;
    }
}

pub(self) use sealed::*;

macro_rules! implement_op {

    (($x:literal, multi)) => {
    paste! {pub struct [<Op $x>];}
    paste! {
        impl Multi for [<Op $x>] {
        type OutputType = [u8; $x*2 / 8];
    }}
    paste!{
    impl RsaMode for [<Op $x>] {
        type InputType = [u8; $x / 8];
    }}
    };

    (($x:literal)) => {
        paste! {pub struct [<Op $x>];}
    paste!{
    impl RsaMode for [<Op $x>] {
        type InputType =  [u8; $x / 8];
    }}
     };

    ($x:tt, $($y:tt),+) => {
        implement_op!($x);
        implement_op!($($y),+);
    };
}

pub(self) use implement_op;

/// Support for RSA peripheral's modular exponentiation feature that could be
/// used to find the `(base ^ exponent) mod modulus`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaModularExponentiation<'a, 'd, T: RsaMode> {
    rsa: &'a mut Rsa<'d>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T: RsaMode, const N: usize> RsaModularExponentiation<'a, 'd, T>
where
    T: RsaMode<InputType = [u8; N]>,
{
    /// starts the modular exponentiation operation. `r` could be calculated
    /// using `2 ^ ( bitlength * 2 ) mod modulus`, for more information
    /// check 24.3.2 in the <https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf>
    pub fn start_exponentiation(&mut self, base: &T::InputType, r: &T::InputType) {
        unsafe {
            self.rsa.write_operand_a(base);
            self.rsa.write_r(r);
        }
        self.set_start();
    }

    /// reads the result to the given buffer.
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully. `start_exponentiation` must be
    /// called before calling this function.
    pub fn read_results(&mut self, outbuf: &mut T::InputType) -> nb::Result<(), Infallible> {
        if !self.rsa.is_idle() {
            return Err(nb::Error::WouldBlock);
        }
        unsafe {
            self.rsa.read_out(outbuf);
        }
        self.rsa.clear_interrupt();
        Ok(())
    }
}

/// Support for RSA peripheral's modular multiplication feature that could be
/// used to find the `(operand a * operand b) mod modulus`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaModularMultiplication<'a, 'd, T: RsaMode> {
    rsa: &'a mut Rsa<'d>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T: RsaMode, const N: usize> RsaModularMultiplication<'a, 'd, T>
where
    T: RsaMode<InputType = [u8; N]>,
{
    /// Reads the result to the given buffer.
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully.
    pub fn read_results(&mut self, outbuf: &mut T::InputType) -> nb::Result<(), Infallible> {
        if !self.rsa.is_idle() {
            return Err(nb::Error::WouldBlock);
        }
        unsafe {
            self.rsa.read_out(outbuf);
        }
        self.rsa.clear_interrupt();
        Ok(())
    }
}

/// Support for RSA peripheral's large number multiplication feature that could
/// be used to find the `operand a * operand b`.
///
/// Each operand is a little endian byte array of the same size
pub struct RsaMultiplication<'a, 'd, T: RsaMode + Multi> {
    rsa: &'a mut Rsa<'d>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T: RsaMode + Multi, const N: usize> RsaMultiplication<'a, 'd, T>
where
    T: RsaMode<InputType = [u8; N]>,
{
    /// Reads the result to the given buffer.
    /// This is a non blocking function that returns without an error if
    /// operation is completed successfully. `start_multiplication` must be
    /// called before calling this function.
    pub fn read_results<'b, const O: usize>(
        &mut self,
        outbuf: &mut T::OutputType,
    ) -> nb::Result<(), Infallible>
    where
        T: Multi<OutputType = [u8; O]>,
    {
        if !self.rsa.is_idle() {
            return Err(nb::Error::WouldBlock);
        }
        unsafe {
            self.rsa.read_out(outbuf);
        }
        self.rsa.clear_interrupt();
        Ok(())
    }
}
