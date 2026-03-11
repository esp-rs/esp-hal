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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RiscvFlavour {
    Basic,
    Plic,
    Clic,
}

impl RiscvFlavour {
    /// Interrupt lines reserved by hardware.
    fn reserved_interrupts(&self) -> impl Iterator<Item = usize> {
        let reserved: &[_] = match self {
            RiscvFlavour::Basic => &[
                0, // Permanently disabled
                1, // Either disabled or reserved for Wi-Fi, unclear
            ],
            RiscvFlavour::Plic => {
                // Some CLINT interrupts
                &[0, 3, 4, 7]
            }
            RiscvFlavour::Clic => {
                // All CLINT interrupts
                &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
            }
        };
        reserved.iter().copied()
    }

    fn disabled_interrupt(&self) -> usize {
        match *self {
            RiscvFlavour::Plic => 31, // 0 is U-mode software interrupt
            RiscvFlavour::Basic | RiscvFlavour::Clic => 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct RiscvControllerProperties {
    flavour: RiscvFlavour,
    /// Total number of interrupts supported by the controller.
    interrupts: u32,
    /// Priority levels above 0
    priority_levels: u32,
}

impl RiscvControllerProperties {
    /// Interrupt lines allocated for vectored interrupt handling.
    fn vector_interrupts(&self) -> impl Iterator<Item = usize> {
        let vectors = match self.flavour {
            RiscvFlavour::Basic | RiscvFlavour::Plic => {
                // Vectoring uses high interrupt lines, as higher IDs are serviced later.
                // Allocate the last interrupt lines, that are not reserved or disabled.
                let mut interrupts = (0..self.interrupts as usize)
                    .rev()
                    .filter(|&intr| {
                        self.disabled_interrupt() != intr
                            && self.reserved_interrupts().all(|reserved| reserved != intr)
                    })
                    .take(self.priority_levels as usize)
                    .collect::<Vec<_>>();

                // We want to return an ascending order
                interrupts.reverse();

                interrupts
            }
            RiscvFlavour::Clic => {
                // After CLINT interrupts. Lower IDs are serviced later.
                // Controller implements proper level/priority management.
                (16..(16 + self.priority_levels as usize)).collect::<Vec<_>>()
            }
        };

        vectors.into_iter()
    }

    fn disabled_interrupt(&self) -> usize {
        self.flavour.disabled_interrupt()
    }

    fn reserved_interrupts(&self) -> impl Iterator<Item = usize> {
        self.flavour.reserved_interrupts()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum InterruptControllerProperties {
    Xtensa,
    Riscv(RiscvControllerProperties),
}

impl GenericProperty for InterruptControllerProperties {
    fn macros(&self) -> Option<TokenStream> {
        let Self::Riscv(properties) = self else {
            return None;
        };

        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        enum Class {
            Interrupt,
            Reserved,
            Vector,
            Disabled,
        }

        // interrupt number => class
        let mut classes = (0..properties.interrupts)
            .map(|_| Class::Interrupt)
            .collect::<Vec<_>>();

        // Implementation assumes contiguous range of interrupts
        assert!(is_contiguous(properties.vector_interrupts()));

        for intr in properties.vector_interrupts() {
            assert_eq!(classes[intr], Class::Interrupt);
            classes[intr] = Class::Vector;
        }
        for intr in properties.reserved_interrupts() {
            assert_eq!(classes[intr], Class::Interrupt);
            classes[intr] = Class::Reserved;
        }
        assert_ne!(classes[properties.disabled_interrupt()], Class::Vector);
        classes[properties.disabled_interrupt()] = Class::Disabled;

        let mut all = vec![];
        let mut vector = vec![];
        let mut reserved = vec![];
        let mut direct_bindable = vec![];

        for (i, class) in classes.iter().enumerate() {
            let intr_number = number(i);
            let idx_in_class = number(match class {
                Class::Interrupt => direct_bindable.len(),
                Class::Vector => vector.len(),
                Class::Reserved => reserved.len(),
                Class::Disabled => 0,
            });
            let class_name = match class {
                Class::Interrupt => format_ident!("direct_bindable"),
                Class::Vector => format_ident!("vector"),
                Class::Reserved => format_ident!("reserved"),
                Class::Disabled => format_ident!("disabled"),
            };

            let tokens = quote! { [#class_name #idx_in_class] #intr_number };

            all.push(tokens.clone());
            match class {
                Class::Interrupt => direct_bindable.push(tokens),
                Class::Vector => vector.push(tokens),
                Class::Reserved => reserved.push(tokens),
                Class::Disabled => {}
            }
        }

        let for_each_interrupt = generate_for_each_macro("interrupt", &[("all", &all)]);
        let for_each_direct_bindable_interrupt = generate_for_each_macro(
            "classified_interrupt",
            &[
                ("direct_bindable", &direct_bindable),
                ("vector", &vector),
                ("reserved", &reserved),
            ],
        );

        let all_priorities = (0..properties.priority_levels)
            .map(|p| {
                let variant = format_ident!("Priority{}", p + 1);
                let numeric = number(p + 1);
                let p = number(p);

                quote! {
                    #p, #numeric, #variant
                }
            })
            .collect::<Vec<_>>();

        let for_each_priority =
            generate_for_each_macro("interrupt_priority", &[("all", &all_priorities)]);

        Some(quote! {
            #for_each_interrupt
            #for_each_direct_bindable_interrupt
            #for_each_priority
        })
    }

    fn cfgs(&self) -> Option<Vec<String>> {
        let controller = match self {
            Self::Xtensa => "xtensa",
            Self::Riscv(properties) => match properties.flavour {
                RiscvFlavour::Basic => "riscv_basic",
                RiscvFlavour::Plic => "plic",
                RiscvFlavour::Clic => "clic",
            },
        };

        Some(vec![format!("interrupt_controller=\"{controller}\"")])
    }

    fn property_macro_branches(&self) -> proc_macro2::TokenStream {
        match self {
            Self::Xtensa => quote! {},
            Self::Riscv(properties) => {
                let disabled = number(properties.disabled_interrupt());
                quote::quote! {
                    ("interrupts.disabled_interrupt") => {
                        #disabled
                    };
                }
            }
        }
    }
}

fn is_contiguous(mut iter: impl Iterator<Item = usize>) -> bool {
    if let Some(mut prev) = iter.next() {
        for next in iter {
            if next - prev != 1 {
                return false;
            }
            prev = next;
        }
    }
    true
}
