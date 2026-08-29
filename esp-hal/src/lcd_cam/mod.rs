//! # LCD and Camera
//!
//! ## Overview
//! This peripheral consists of an LCD module and a Camera module, which can be
//! used simultaneously. For more information on these modules, please refer to
//! the documentation in their respective modules.

use core::marker::PhantomData;

use enumset::{EnumSet, EnumSetType};

use crate::{
    Async,
    Blocking,
    asynch::AtomicWaker,
    clock::dividers::FractionalDivider,
    handler,
    interrupt::InterruptHandler,
    lcd_cam::{cam::Cam, lcd::Lcd},
    peripherals::{Interrupt, LCD_CAM},
    system::{Cpu, GenericPeripheralGuard},
};

pub mod cam;
pub mod lcd;
pub(crate) mod ll;

/// DMA TX channel trait for LCD (I8080, DPI) peripherals.
///
/// Implemented for every TX-capable channel type that can serve the LCD module.
#[diagnostic::on_unimplemented(
    message = "The DMA channel cannot be used as a TX channel for LCD",
    label = "This DMA channel"
)]
pub trait LcdDmaTxChannel<'d>: Into<ErasedTxChannel<'d>> + crate::private::Sealed {}

/// DMA RX channel trait for the Camera peripheral.
///
/// Implemented for every RX-capable channel type that can serve the Camera module.
#[diagnostic::on_unimplemented(
    message = "The DMA channel cannot be used as an RX channel for Camera",
    label = "This DMA channel"
)]
pub trait CamDmaRxChannel<'d>: Into<ErasedRxChannel<'d>> + crate::private::Sealed {}

with_lcd_cam_dma_engine! {
    ($engine:tt, $any_channel:tt) => {
        type ErasedTxChannel<'d> = <crate::dma::$any_channel<'d> as crate::dma::DmaChannel>::Tx;
        type ErasedRxChannel<'d> = <crate::dma::$any_channel<'d> as crate::dma::DmaChannel>::Rx;

        crate::macros::impl_dma_channel_trait! {
            $engine,
            peri = LCD_CAM,
            ($peri:path, $ch:path) => {
                impl<'d> LcdDmaTxChannel<'d> for $ch {}
                impl<'d> CamDmaRxChannel<'d> for $ch {}
            }
        }

        // All channels split into the erased TX/RX channels, so we
        // must implement the traits only once, outside of the macro.
        impl<'d> LcdDmaTxChannel<'d> for ErasedTxChannel<'d> {}
        impl<'d> CamDmaRxChannel<'d> for ErasedRxChannel<'d> {}
    };
}

/// Represents a combined LCD and Camera interface.
pub struct LcdCam<'d, Dm: crate::DriverMode> {
    /// The LCD interface.
    pub lcd: Lcd<'d, Dm>,
    /// The Camera interface.
    pub cam: Cam<'d>,
}

impl<'d> LcdCam<'d, Blocking> {
    /// Creates a new `LcdCam` instance.
    pub fn new(lcd_cam: LCD_CAM<'d>) -> Self {
        let lcd_guard = GenericPeripheralGuard::new();
        let cam_guard = GenericPeripheralGuard::new();

        Self {
            lcd: Lcd {
                inner: lcd::Inner {
                    lcd_cam: unsafe { lcd_cam.clone_unchecked() },
                    _guard: lcd_guard,
                    clock_requested: false,
                },
                _mode: PhantomData,
            },
            cam: Cam {
                lcd_cam,
                _guard: cam_guard,
                clock_requested: false,
            },
        }
    }

    /// Reconfigures the peripheral for asynchronous operation.
    pub fn into_async(mut self) -> LcdCam<'d, Async> {
        self.set_interrupt_handler(interrupt_handler);
        LcdCam {
            lcd: self.lcd.into_async(),
            cam: self.cam,
        }
    }

    /// Registers an interrupt handler for the LCD_CAM peripheral.
    ///
    /// Replaces any previously registered interrupt handlers.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::LCD_CAM);
        }
        crate::interrupt::bind_handler(Interrupt::LCD_CAM, handler);
    }
}

impl crate::private::Sealed for LcdCam<'_, Blocking> {}
// TODO: This interrupt is shared with the Camera module, we should handle this
// in a similar way to the gpio::IO
#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for LcdCam<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

impl<'d> LcdCam<'d, Async> {
    /// Reconfigures the peripheral for blocking operation.
    pub fn into_blocking(self) -> LcdCam<'d, Blocking> {
        crate::interrupt::disable(Cpu::current(), Interrupt::LCD_CAM);
        LcdCam {
            lcd: self.lcd.into_blocking(),
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
    /// Inverts bit order.
    Inverted = 1,
}

/// LCD_CAM byte order
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ByteOrder {
    /// Do not change byte order.
    #[default]
    Native   = 0,
    /// Inverts byte order.
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

/// The interrupt sources of the LCD_CAM DMA interrupt registers that belong to
/// the LCD half of the peripheral.
///
/// This type is `pub(crate)` today. If the Camera driver ever needs its own
/// listen API, it can be promoted to a public `LcdCamInterrupt` without
/// renaming.
#[derive(Debug, EnumSetType)]
pub(crate) enum LcdCamInterrupt {
    /// The LCD has started outputting a new frame.
    LcdVsync,

    /// A DMA transfer to the LCD has finished.
    LcdTransDone,
}

// NOTE: the LCD_CAM interrupt registers are shared between LCD and Camera and
// this is only implemented for the LCD side, when the Camera is implemented a
// CriticalSection will be needed to protect these shared registers.
impl Instance {
    fn enable_listen(sources: EnumSet<LcdCamInterrupt>, en: bool) {
        LCD_CAM::regs().lc_dma_int_ena().modify(|_, w| {
            for source in sources {
                match source {
                    LcdCamInterrupt::LcdVsync => {
                        w.lcd_vsync_int_ena().bit(en);
                    }
                    LcdCamInterrupt::LcdTransDone => {
                        w.lcd_trans_done_int_ena().bit(en);
                    }
                }
            }
            w
        });
    }

    pub(crate) fn listen(sources: EnumSet<LcdCamInterrupt>) {
        Self::enable_listen(sources, true);
    }

    pub(crate) fn unlisten(sources: EnumSet<LcdCamInterrupt>) {
        Self::enable_listen(sources, false);
    }

    pub(crate) fn interrupts() -> EnumSet<LcdCamInterrupt> {
        let raw = LCD_CAM::regs().lc_dma_int_raw().read();
        let mut sources = EnumSet::new();
        if raw.lcd_vsync_int_raw().bit() {
            sources.insert(LcdCamInterrupt::LcdVsync);
        }
        if raw.lcd_trans_done_int_raw().bit() {
            sources.insert(LcdCamInterrupt::LcdTransDone);
        }
        sources
    }

    /// Only reachable through the unstable `I8080` interrupt API, hence the
    /// `dead_code` allowance when the `unstable` feature is disabled.
    #[cfg_attr(not(feature = "unstable"), allow(dead_code))]
    pub(crate) fn clear_interrupts(sources: EnumSet<LcdCamInterrupt>) {
        LCD_CAM::regs().lc_dma_int_clr().write(|w| {
            for source in sources {
                match source {
                    LcdCamInterrupt::LcdVsync => {
                        w.lcd_vsync_int_clr().set_bit();
                    }
                    LcdCamInterrupt::LcdTransDone => {
                        w.lcd_trans_done_int_clr().set_bit();
                    }
                }
            }
            w
        });
    }

    pub(crate) fn listen_lcd_done() {
        Self::listen(LcdCamInterrupt::LcdTransDone.into());
    }

    pub(crate) fn unlisten_lcd_done() {
        Self::unlisten(LcdCamInterrupt::LcdTransDone.into());
    }

    pub(crate) fn is_lcd_done_set() -> bool {
        Self::interrupts().contains(LcdCamInterrupt::LcdTransDone)
    }
}
pub(crate) struct ClockDivider {
    /// Integral clock divider value, 2 to 256.
    pub div_num: u32,

    /// Fractional clock divider numerator value, 0 to 63.
    pub div_b: u32,

    /// Fractional clock divider denominator value, 1 to 63.
    pub div_a: u32,
}

impl ClockDivider {
    fn new(divider: FractionalDivider) -> Self {
        Self {
            div_num: divider.integer,
            div_b: divider.numerator,
            // An integral divider has no denominator, but the clock tree only accepts
            // denominators of 1 or more.
            div_a: divider.denominator.max(1),
        }
    }
}

/// Clock configuration errors.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockError {
    /// Desired frequency was too low for the dividers to divide to.
    FrequencyTooLow,
}

pub(crate) fn calculate_clkm(
    desired_frequency: u32,
    source_frequencies: &[u32],
) -> Result<(usize, ClockDivider), ClockError> {
    let mut result_error = 0;
    let mut result = None;

    for (i, &source_frequency) in source_frequencies.iter().enumerate() {
        let Some(divider) = calculate_closest_divider(source_frequency, desired_frequency) else {
            continue;
        };

        // A divider may land either side of the desired frequency, so pick the source that gets
        // closest to it.
        let error = divider
            .output_frequency(source_frequency)
            .abs_diff(desired_frequency);
        if result.is_none() || error < result_error {
            result = Some((i, divider));
            result_error = error;
        }
    }

    let (index, divider) = result.ok_or(ClockError::FrequencyTooLow)?;

    Ok((index, ClockDivider::new(divider)))
}

fn calculate_closest_divider(
    source_frequency: u32,
    desired_frequency: u32,
) -> Option<FractionalDivider> {
    // For current chips, LCD and CAM have the same divider range.
    let (min_divider, max_divider) = property!("clock_tree.lcd_cam.lcd_clock.div_num");
    let (_, max_denominator) = property!("clock_tree.lcd_cam.lcd_clock.div_a");

    if source_frequency / desired_frequency < min_divider {
        // Source clock isn't fast enough to reach the desired frequency.
        // Return max output.
        return Some(FractionalDivider {
            integer: min_divider,
            numerator: 0,
            denominator: 0,
        });
    }

    let divider = FractionalDivider::new(source_frequency, desired_frequency, max_denominator);

    // Source is too fast to divide down to the desired frequency.
    (divider.integer <= max_divider).then_some(divider)
}
