use proc_macro2::TokenStream;
use quote::{format_ident, quote};

use crate::{cfg::GenericProperty, generate_for_each_macro, number};

#[derive(Debug, Clone, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct SoftwareInterruptProperties {
    #[serde(rename = "software_interrupt_count")]
    count: u32,
    #[serde(rename = "software_interrupt_delay")]
    delay: u32,
}

/// Generates `for_each_sw_interrupt!` which can be used to implement SoftwareInterruptControl, and
/// `sw_interrupt_delay` which repeats `nop` enough times to ensure the interrupt is fired before
/// returning.
impl GenericProperty for SoftwareInterruptProperties {
    fn macros(&self) -> Option<TokenStream> {
        let nops =
            std::iter::repeat(quote! { ::core::arch::asm!("nop"); }).take(self.delay as usize);

        let channels = (0..self.count)
            .map(|i| {
                let idx = number(i);
                let interrupt = format_ident!("FROM_CPU_INTR{}", i);
                let field = format_ident!("software_interrupt{}", i);
                quote! { #idx, #interrupt, #field }
            })
            .collect::<Vec<_>>();

        let for_each_sw_interrupt = generate_for_each_macro("sw_interrupt", &[("all", &channels)]);

        Some(quote! {
            #for_each_sw_interrupt

            #[macro_export]
            macro_rules! sw_interrupt_delay {
                () => {
                    unsafe {
                        #(#nops)*
                    }
                };
            }
        })
    }
}
