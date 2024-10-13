//! # LCD and Camera
//!
//! ## Overview
//! This peripheral consists of an LCD module and a Camera module, which can be
//! used simultaneously. For more information on these modules, please refer to
//! the documentation in their respective modules.

pub mod cam;
pub mod lcd;

use core::marker::PhantomData;

use enumset::{EnumSet, EnumSetType};

use crate::{
    interrupt::InterruptHandler,
    lcd_cam::{cam::Cam, lcd::Lcd},
    peripheral::Peripheral,
    peripherals::LCD_CAM,
    system::{self, PeripheralClockControl},
    Blocking,
    InterruptConfigurable,
};

/// Represents a combined LCD and Camera interface.
pub struct LcdCam<'d, M = BlockingWithInterrupts> {
    /// The LCD interface.
    pub lcd: Lcd<'d, M>,
    /// The Camera interface.
    pub cam: Cam<'d>,
}

impl<'d> LcdCam<'d, BlockingWithInterrupts> {
    /// Creates a new `LcdCam` instance.
    pub fn new(lcd_cam: impl Peripheral<P = LCD_CAM> + 'd) -> Self {
        crate::into_ref!(lcd_cam);

        PeripheralClockControl::reset(system::Peripheral::LcdCam);
        PeripheralClockControl::enable(system::Peripheral::LcdCam);

        Self {
            lcd: Lcd {
                lcd_cam: unsafe { lcd_cam.clone_unchecked() },
                mode: PhantomData,
            },
            cam: Cam {
                lcd_cam: unsafe { lcd_cam.clone_unchecked() },
            },
        }
    }

    /// Split out the [InterruptControl] from the driver.
    pub fn split_interrupts(self) -> (LcdCam<'d, Blocking>, InterruptControl<'d>) {
        let interrupts = unsafe { InterruptControl::steal() };
        (
            LcdCam {
                lcd: Lcd {
                    lcd_cam: self.lcd.lcd_cam,
                    mode: PhantomData,
                },
                cam: self.cam,
            },
            interrupts,
        )
    }
}

/// Types of interrupts emitted by the LCD_CAM.
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LcdCamInterrupt {
    /// Triggered when the total number of received lines by camera is greater
    /// than or equal to LCD_CAM_CAM_LINE_INT_NUM + 1.
    CamHsync,

    /// Triggered when the camera received a VSYNC signal.
    CamVsync,

    /// Triggered when the LCD transmitted all the data. (Only relevant in I8080
    /// mode)
    LcdTransDone,

    /// Triggered when the LCD transmitted a VSYNC signal. (Only relevant in RGB
    /// mode)
    LcdVsync,
}

/// Access to the interrupt bits of the LCD_CAM.
pub struct InterruptControl<'d>(PhantomData<&'d ()>);

impl<'d> InterruptControl<'d> {
    /// Listen for the given interrupts.
    pub fn listen(&self, interrupts: impl Into<EnumSet<LcdCamInterrupt>>) {
        self.enable_listen(interrupts.into(), true)
    }

    /// Stop listening for the given interrupts.
    pub fn unlisten(&self, interrupts: impl Into<EnumSet<LcdCamInterrupt>>) {
        self.enable_listen(interrupts.into(), false)
    }

    fn enable_listen(&self, interrupts: EnumSet<LcdCamInterrupt>, enable: bool) {
        let lcd_cam = unsafe { LCD_CAM::steal() };
        lcd_cam.lc_dma_int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    LcdCamInterrupt::CamHsync => w.cam_hs_int_ena().bit(enable),
                    LcdCamInterrupt::CamVsync => w.cam_vsync_int_ena().bit(enable),
                    LcdCamInterrupt::LcdTransDone => w.lcd_trans_done_int_ena().bit(enable),
                    LcdCamInterrupt::LcdVsync => w.lcd_vsync_int_ena().bit(enable),
                };
            }
            w
        })
    }

    /// Reset the given interrupts.
    pub fn clear(&self, interrupts: impl Into<EnumSet<LcdCamInterrupt>>) {
        let interrupts = interrupts.into();

        let lcd_cam = unsafe { esp32s3::LCD_CAM::steal() };
        lcd_cam.lc_dma_int_clr().write(|w| {
            for interrupt in interrupts {
                match interrupt {
                    LcdCamInterrupt::CamHsync => w.cam_hs_int_clr().set_bit(),
                    LcdCamInterrupt::CamVsync => w.cam_vsync_int_clr().set_bit(),
                    LcdCamInterrupt::LcdTransDone => w.lcd_trans_done_int_clr().set_bit(),
                    LcdCamInterrupt::LcdVsync => w.lcd_vsync_int_clr().set_bit(),
                };
            }
            w
        });
    }

    /// Returns the set of interrupts being listened to.
    pub fn is_listening(&self) -> EnumSet<LcdCamInterrupt> {
        let lcd_cam = unsafe { LCD_CAM::steal() };

        let int_ena = lcd_cam.lc_dma_int_ena().read();

        let mut result = EnumSet::empty();

        if int_ena.cam_hs_int_ena().bit_is_set() {
            result |= LcdCamInterrupt::CamHsync;
        }
        if int_ena.cam_vsync_int_ena().bit_is_set() {
            result |= LcdCamInterrupt::CamVsync;
        }
        if int_ena.lcd_trans_done_int_ena().bit_is_set() {
            result |= LcdCamInterrupt::LcdTransDone;
        }
        if int_ena.lcd_vsync_int_ena().bit_is_set() {
            result |= LcdCamInterrupt::LcdVsync;
        }

        result
    }

    /// Returns the set of asserted interrupts.
    pub fn pending_interrupts(&self) -> EnumSet<LcdCamInterrupt> {
        let lcd_cam = unsafe { LCD_CAM::steal() };

        let int_raw = lcd_cam.lc_dma_int_raw().read();

        let mut result = EnumSet::empty();

        if int_raw.cam_hs_int_raw().bit_is_set() {
            result |= LcdCamInterrupt::CamHsync;
        }
        if int_raw.cam_vsync_int_raw().bit_is_set() {
            result |= LcdCamInterrupt::CamVsync;
        }
        if int_raw.lcd_trans_done_int_raw().bit_is_set() {
            result |= LcdCamInterrupt::LcdTransDone;
        }
        if int_raw.lcd_vsync_int_raw().bit_is_set() {
            result |= LcdCamInterrupt::LcdVsync;
        }

        result
    }

    /// Returns the set of asserted interrupts that are being listened to.
    pub fn active_interrupts(&self) -> EnumSet<LcdCamInterrupt> {
        let lcd_cam = unsafe { LCD_CAM::steal() };

        let int_st = lcd_cam.lc_dma_int_st().read();

        let mut result = EnumSet::empty();

        if int_st.cam_hs_int_st().bit_is_set() {
            result |= LcdCamInterrupt::CamHsync;
        }
        if int_st.cam_vsync_int_st().bit_is_set() {
            result |= LcdCamInterrupt::CamVsync;
        }
        if int_st.lcd_trans_done_int_st().bit_is_set() {
            result |= LcdCamInterrupt::LcdTransDone;
        }
        if int_st.lcd_vsync_int_st().bit_is_set() {
            result |= LcdCamInterrupt::LcdVsync;
        }

        result
    }

    /// Unsafely create an instance of this type.
    ///
    /// # Safety
    ///
    /// The caller must ensure that only one instance of this type is in use at
    /// one time.
    pub unsafe fn steal() -> Self {
        Self(PhantomData)
    }
}

impl<'d> crate::private::Sealed for InterruptControl<'d> {}

impl<'d> InterruptConfigurable for InterruptControl<'d> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        unsafe {
            crate::interrupt::bind_interrupt(
                crate::peripherals::Interrupt::LCD_CAM,
                handler.handler(),
            );
            crate::interrupt::enable(crate::peripherals::Interrupt::LCD_CAM, handler.priority())
                .unwrap();
        }
    }
}

/// Same as [Blocking] but with interrupts access.
pub struct BlockingWithInterrupts;

/// LCD_CAM bit order
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BitOrder {
    /// Do not change bit order.
    #[default]
    Native   = 0,
    /// Invert bit order.
    Inverted = 1,
}

/// LCD_CAM byte order
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ByteOrder {
    /// Do not change bit order.
    #[default]
    Native   = 0,
    /// Invert byte order.
    Inverted = 1,
}

#[doc(hidden)]
pub mod asynch {
    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::handler;

    use crate::lcd_cam::InterruptControl;

    pub(crate) static WAKER: AtomicWaker = AtomicWaker::new();

    #[handler]
    pub(crate) fn interrupt_handler() {
        let int_access = unsafe { InterruptControl::steal() };

        let active_interrupts = int_access.active_interrupts();
        if !active_interrupts.is_empty() {
            int_access.unlisten(active_interrupts);
            WAKER.wake();
        }
    }
}

mod private {
    use crate::{dma::PeripheralMarker, peripherals::LCD_CAM};

    impl PeripheralMarker for LCD_CAM {
        fn peripheral(&self) -> crate::system::Peripheral {
            crate::system::Peripheral::LcdCam
        }
    }

    pub struct ClockDivider {
        // Integral LCD clock divider value. (8 bits)
        // Value 0 is treated as 256
        // Value 1 is treated as 2
        // Value N is treated as N
        pub div_num: usize,

        // Fractional clock divider numerator value. (6 bits)
        pub div_b: usize,

        // Fractional clock divider denominator value. (6 bits)
        pub div_a: usize,
    }

    pub fn calculate_clkm(
        desired_frequency: usize,
        source_frequencies: &[usize],
    ) -> (usize, ClockDivider) {
        let mut result_freq = 0;
        let mut result = None;

        for (i, &source_frequency) in source_frequencies.iter().enumerate() {
            let div = calculate_closest_divider(source_frequency, desired_frequency);
            if let Some(div) = div {
                let freq = calculate_output_frequency(source_frequency, &div);
                if result.is_none() || freq > result_freq {
                    result = Some((i, div));
                    result_freq = freq;
                }
            }
        }

        result.expect("Desired frequency was too low for the dividers to divide to")
    }

    fn calculate_output_frequency(source_frequency: usize, divider: &ClockDivider) -> usize {
        let n = match divider.div_num {
            0 => 256,
            1 => 2,
            _ => divider.div_num.min(256),
        };

        if divider.div_b != 0 && divider.div_a != 0 {
            // OUTPUT = SOURCE / (N + B/A)
            // OUTPUT = SOURCE / ((NA + B)/A)
            // OUTPUT = (SOURCE * A) / (NA + B)

            // u64 is required to fit the numbers from this arithmetic.

            let source = source_frequency as u64;
            let n = n as u64;
            let a = divider.div_b as u64;
            let b = divider.div_a as u64;

            ((source * a) / (n * a + b)) as _
        } else {
            source_frequency / n
        }
    }

    fn calculate_closest_divider(
        source_frequency: usize,
        desired_frequency: usize,
    ) -> Option<ClockDivider> {
        let div_num = source_frequency / desired_frequency;
        if div_num < 2 {
            // Source clock isn't fast enough to reach the desired frequency.
            // Return max output.
            return Some(ClockDivider {
                div_num: 1,
                div_b: 0,
                div_a: 0,
            });
        }
        if div_num > 256 {
            // Source is too fast to divide to the desired frequency. Return None.
            return None;
        }

        let div_num = if div_num == 256 { 0 } else { div_num };

        let div_fraction = {
            let div_remainder = source_frequency % desired_frequency;
            let gcd = hcf(div_remainder, desired_frequency);
            Fraction {
                numerator: div_remainder / gcd,
                denominator: desired_frequency / gcd,
            }
        };

        let divider = if div_fraction.numerator == 0 {
            ClockDivider {
                div_num,
                div_b: 0,
                div_a: 0,
            }
        } else {
            let target = div_fraction;
            let closest = farey_sequence(63)
                .find(|curr| {
                    // https://en.wikipedia.org/wiki/Fraction#Adding_unlike_quantities

                    let new_curr_num = curr.numerator * target.denominator;
                    let new_target_num = target.numerator * curr.denominator;
                    new_curr_num >= new_target_num
                })
                .expect("The fraction must be between 0 and 1");

            ClockDivider {
                div_num,
                div_b: closest.numerator,
                div_a: closest.denominator,
            }
        };
        Some(divider)
    }

    // https://en.wikipedia.org/wiki/Euclidean_algorithm
    const fn hcf(a: usize, b: usize) -> usize {
        if b != 0 {
            hcf(b, a % b)
        } else {
            a
        }
    }

    struct Fraction {
        pub numerator: usize,
        pub denominator: usize,
    }

    // https://en.wikipedia.org/wiki/Farey_sequence#Next_term
    fn farey_sequence(denominator: usize) -> impl Iterator<Item = Fraction> {
        let mut a = 0;
        let mut b = 1;
        let mut c = 1;
        let mut d = denominator;
        core::iter::from_fn(move || {
            if a > denominator {
                return None;
            }
            let next = Fraction {
                numerator: a,
                denominator: b,
            };
            let k = (denominator + b) / d;
            (a, b, c, d) = (c, d, k * c - a, k * d - b);
            Some(next)
        })
    }
}
