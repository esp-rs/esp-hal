//! # LCD and Camera
//!
//! ## Overview
//! This peripheral consists of an LCD module and a Camera module, which can be
//! used simultaneously. For more information on these modules, please refer to
//! the documentation in their respective modules.

pub mod cam;
pub mod lcd;

use core::marker::PhantomData;

use crate::{
    asynch::AtomicWaker,
    interrupt::{InterruptConfigurable, InterruptHandler},
    lcd_cam::{cam::Cam, lcd::Lcd},
    macros::handler,
    peripheral::Peripheral,
    peripherals::{Interrupt, LCD_CAM},
    system::GenericPeripheralGuard,
    Async,
    Blocking,
    Cpu,
};

/// Represents a combined LCD and Camera interface.
pub struct LcdCam<'d, DM: crate::Mode> {
    /// The LCD interface.
    pub lcd: Lcd<'d, DM>,
    /// The Camera interface.
    pub cam: Cam<'d>,
}

impl<'d> LcdCam<'d, Blocking> {
    /// Creates a new `LcdCam` instance.
    pub fn new(lcd_cam: impl Peripheral<P = LCD_CAM> + 'd) -> Self {
        crate::into_ref!(lcd_cam);

        let lcd_guard = GenericPeripheralGuard::new();
        let cam_guard = GenericPeripheralGuard::new();

        Self {
            lcd: Lcd {
                lcd_cam: unsafe { lcd_cam.clone_unchecked() },
                _mode: PhantomData,
                _guard: lcd_guard,
            },
            cam: Cam {
                lcd_cam,
                _guard: cam_guard,
            },
        }
    }

    /// Reconfigures the peripheral for asynchronous operation.
    pub fn into_async(mut self) -> LcdCam<'d, Async> {
        self.set_interrupt_handler(interrupt_handler);
        LcdCam {
            lcd: Lcd {
                lcd_cam: self.lcd.lcd_cam,
                _mode: PhantomData,
                _guard: self.lcd._guard,
            },
            cam: self.cam,
        }
    }
}

impl crate::private::Sealed for LcdCam<'_, Blocking> {}
// TODO: This interrupt is shared with the Camera module, we should handle this
// in a similar way to the gpio::IO
impl InterruptConfigurable for LcdCam<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in crate::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::LCD_CAM);
        }
        unsafe { crate::interrupt::bind_interrupt(Interrupt::LCD_CAM, handler.handler()) };
        unwrap!(crate::interrupt::enable(
            Interrupt::LCD_CAM,
            handler.priority()
        ));
    }
}

impl<'d> LcdCam<'d, Async> {
    /// Reconfigures the peripheral for blocking operation.
    pub fn into_blocking(self) -> LcdCam<'d, Blocking> {
        crate::interrupt::disable(Cpu::current(), Interrupt::LCD_CAM);
        LcdCam {
            lcd: Lcd {
                lcd_cam: self.lcd.lcd_cam,
                _mode: PhantomData,
                _guard: self.lcd._guard,
            },
            cam: self.cam,
        }
    }
}

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
    /// Do not change byte order.
    #[default]
    Native   = 0,
    /// Invert byte order.
    Inverted = 1,
}

pub(crate) static LCD_DONE_WAKER: AtomicWaker = AtomicWaker::new();

#[handler]
fn interrupt_handler() {
    // TODO: this is a shared interrupt with Camera and here we ignore that!
    if Instance::is_lcd_done_set() {
        Instance::unlisten_lcd_done();
        LCD_DONE_WAKER.wake()
    }
}

pub(crate) struct Instance;

// NOTE: the LCD_CAM interrupt registers are shared between LCD and Camera and
// this is only implemented for the LCD side, when the Camera is implemented a
// CriticalSection will be needed to protect these shared registers.
impl Instance {
    pub(crate) fn listen_lcd_done() {
        let lcd_cam = unsafe { LCD_CAM::steal() };
        lcd_cam
            .lc_dma_int_ena()
            .modify(|_, w| w.lcd_trans_done_int_ena().set_bit());
    }

    pub(crate) fn unlisten_lcd_done() {
        let lcd_cam = unsafe { LCD_CAM::steal() };
        lcd_cam
            .lc_dma_int_ena()
            .modify(|_, w| w.lcd_trans_done_int_ena().clear_bit());
    }

    pub(crate) fn is_lcd_done_set() -> bool {
        let lcd_cam = unsafe { LCD_CAM::steal() };
        lcd_cam
            .lc_dma_int_raw()
            .read()
            .lcd_trans_done_int_raw()
            .bit()
    }
}
pub(crate) struct ClockDivider {
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

pub(crate) fn calculate_clkm(
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
