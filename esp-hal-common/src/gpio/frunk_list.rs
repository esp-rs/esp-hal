use super::{IO, IO_MUX, GPIO, InitialPinHList};
use frunk::hlist::Plucker;

impl IO<InitialPinHList> {
    /// Manage pins using `frunk::hlist::HList` type:
    ///
    /// ```no-run
    /// // create list same as regular `new`
    /// let io = IO::hl_new(peripherals.GPIO, peripherals.IO_MUX);
    /// // move pin 4 out of the hl-list of pins, and into a variable
    /// let (pin4, io): (GpioPin<Unknown, 4>, _) = io.pluck_pin::<4>();
    /// ```
    pub fn hl_new(gpio: GPIO, io_mux: IO_MUX) -> Self {
        let pins = gpio.hl_split();
        let io = IO {
            _io_mux: io_mux,
            pins,
        };
        io
    }
}
impl<Ty, L0: Plucker<Ty, L1>, L1> Plucker<Ty, L1> for IO<L0>
{
    type Remainder = IO<<L0 as Plucker<Ty, L1>>::Remainder>;
    fn pluck(self) -> (Ty, IO<<L0 as Plucker<Ty, L1>>::Remainder>) 
    {
        let (val, pins) = self.pins.pluck();
        let res = IO {
            _io_mux: self._io_mux,
            pins,
        };
        (val, res)
    }
}

/// ```
/// #[rustfmt::skip]
/// let expected = frunk::hlist!(
///     GpioPin::<Unknown>, 0>::new(),
///     GpioPin::<Unknown>, 1>::new(),
///     GpioPin::<Unknown>, 2>::new()
/// );
/// assert_eq!(gpio_hlist(0, 1, 2), expected);
/// ```
#[macro_export]
macro_rules! gpiopin_hlist {
    ($($gpionum:expr),+) => {
        frunk::hlist![$(GpioPin::<Unknown, $gpionum>::new()),+]
    };
}
/// ```
/// // This should macro-expand to the same as `Expected`
/// type GeneratedType = gpio_HList!(0, 1, 2);
///
/// // expected expantion
/// #[rustfmt::skip]
/// type ExpectedType = HCons<GpioPin<Unknown, { 0 }>,
///                     HCons<GpioPin<Unknown, { 1 }>,
///                     HCons<GpioPin<Unknown, { 2 }>,
///                     HNil>>>;
///
/// // let's set up a means to put the type-checker to use
/// struct ExpectedMaker;
/// impl ExpectedMaker {
///     fn make() -> ExpectedType {
///         unimplemented!()
///     }
/// }
///
/// fn main() {
///     // validate that the make-method conforms with expected expansion
///     let expected: ExpectedType = ExpectedMaker::make();
///     // ...but if it doesn't conform to the actual expansion, this doc-test fails
///     let test: GeneratedType = ExpectedMaker::make();
/// }
/// ```
#[macro_export]
macro_rules! gpiopin_HList {
    ($($gpionum:expr),+) => {
        frunk::HList![$(GpioPin<Unknown, { $gpionum }>),+]
    };
}
