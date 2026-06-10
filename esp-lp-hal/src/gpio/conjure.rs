use super::{Input, MAX_GPIO_PIN, Output, OutputOpenDrain};

// Used by the `entry` procmacro:
#[doc(hidden)]
pub unsafe fn conjure_output<const PIN: u8>() -> Option<Output<PIN>> {
    if PIN > MAX_GPIO_PIN {
        None
    } else {
        Some(Output::<PIN>::new())
    }
}

// Used by the `entry` procmacro:
#[doc(hidden)]
pub unsafe fn conjure_output_open_drain<const PIN: u8>() -> Option<OutputOpenDrain<PIN>> {
    if PIN > MAX_GPIO_PIN {
        None
    } else {
        Some(OutputOpenDrain::<PIN>::new())
    }
}

// Used by the `entry` procmacro:
#[doc(hidden)]
pub unsafe fn conjure_input<const PIN: u8>() -> Option<Input<PIN>> {
    if PIN > MAX_GPIO_PIN {
        None
    } else {
        Some(Input::<PIN>::new())
    }
}
