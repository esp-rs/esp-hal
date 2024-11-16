#[allow(unused)]
use proc_macro::TokenStream;
use quote::quote;

#[cfg(any(feature = "is-lp-core", feature = "is-ulp-core"))]
pub fn entry(args: TokenStream, input: TokenStream) -> TokenStream {
    use proc_macro2::{Ident, Span};
    use proc_macro_crate::{crate_name, FoundCrate};
    use quote::format_ident;
    use syn::{
        parse::Error,
        parse_macro_input,
        spanned::Spanned,
        FnArg,
        GenericArgument,
        ItemFn,
        PatType,
        PathArguments,
        Type,
    };

    pub(crate) fn make_magic_symbol_name(args: &Vec<&PatType>) -> String {
        let mut res = String::from("__ULP_MAGIC_");
        for &a in args {
            let t = &a.ty;
            let quoted = to_string(t);
            res.push_str(&quoted);
            res.push('$');
        }

        res
    }

    pub(crate) fn simplename(t: &Type) -> String {
        match t {
            Type::Path(p) => p.path.segments.last().unwrap().ident.to_string(),
            _ => String::new(),
        }
    }

    pub(crate) fn extract_pin(ty: &Type) -> u8 {
        let mut res = 255u8;
        if let Type::Path(p) = ty {
            let segment = p.path.segments.last().unwrap();
            if let PathArguments::AngleBracketed(g) = &segment.arguments {
                for arg in &g.args {
                    match arg {
                        GenericArgument::Type(t) => {
                            res = extract_pin(t);
                        }
                        GenericArgument::Const(c) => {
                            res = quote! { #c }.to_string().parse().unwrap();
                        }
                        _ => (),
                    }
                }
            }
        }

        res
    }

    // This is a specialized implementation - won't fit other use-cases
    fn to_string(ty: &Type) -> String {
        let mut res = String::new();
        if let Type::Path(p) = ty {
            let segment = p.path.segments.last().unwrap();
            res.push_str(&segment.ident.to_string());

            if let PathArguments::AngleBracketed(g) = &segment.arguments {
                res.push('<');
                let mut pushed = false;
                for arg in &g.args {
                    if pushed {
                        res.push(',');
                    }

                    match arg {
                        GenericArgument::Type(t) => {
                            pushed = true;
                            res.push_str(&to_string(t));
                        }
                        GenericArgument::Const(c) => {
                            pushed = true;
                            res.push_str(&quote! { #c }.to_string());
                        }
                        _ => (),
                    }
                }
                res.push('>');
            }
        }

        res
    }

    let found_crate = crate_name("esp-lp-hal").expect("esp-lp-hal is present in `Cargo.toml`");
    let hal_crate = match found_crate {
        FoundCrate::Itself => quote!(esp_lp_hal),
        FoundCrate::Name(name) => {
            let ident = Ident::new(&name, Span::call_site());
            quote!( #ident )
        }
    };

    if !args.is_empty() {
        return Error::new(Span::call_site(), "This attribute accepts no arguments")
            .to_compile_error()
            .into();
    }

    let f = parse_macro_input!(input as ItemFn);

    let mut argument_types = Vec::new();
    let mut create_peripheral = Vec::new();

    let mut used_pins: Vec<u8> = Vec::new();

    for (num, arg) in f.sig.inputs.iter().enumerate() {
        let param_name = format_ident!("param{}", num);
        match arg {
            FnArg::Receiver(_) => {
                return Error::new(arg.span(), "invalid argument")
                    .to_compile_error()
                    .into();
            }
            FnArg::Typed(t) => {
                match simplename(&t.ty).as_str() {
                    "Output" => {
                        let pin = extract_pin(&t.ty);
                        if used_pins.contains(&pin) {
                            return Error::new(arg.span(), "duplicate pin")
                                .to_compile_error()
                                .into();
                        }
                        used_pins.push(pin);
                        create_peripheral.push(quote!(
                            let mut #param_name = unsafe { the_hal::gpio::conjure_output().unwrap() };
                        ));
                    }
                    "Input" => {
                        let pin = extract_pin(&t.ty);
                        if used_pins.contains(&pin) {
                            return Error::new(arg.span(), "duplicate pin")
                                .to_compile_error()
                                .into();
                        }
                        used_pins.push(pin);
                        create_peripheral.push(quote!(
                            let mut #param_name = unsafe { the_hal::gpio::conjure_input().unwrap() };
                        ));
                    }
                    "LpUart" => {
                        create_peripheral.push(quote!(
                            let mut #param_name = unsafe { the_hal::uart::conjure() };
                        ));
                    }
                    "LpI2c" => {
                        create_peripheral.push(quote!(
                            let mut #param_name = unsafe { the_hal::i2c::conjure() };
                        ));
                    }
                    _ => {
                        return Error::new(arg.span(), "invalid argument to main")
                            .to_compile_error()
                            .into();
                    }
                }
                argument_types.push(t);
            }
        }
    }

    let magic_symbol_name = make_magic_symbol_name(&argument_types);

    let param_names: Vec<Ident> = argument_types
        .into_iter()
        .enumerate()
        .map(|(num, _)| format_ident!("param{}", num))
        .collect();

    quote!(
        #[allow(non_snake_case)]
        #[export_name = "main"]
        pub fn __risc_v_rt__main() -> ! {
            #[export_name = #magic_symbol_name]
            static ULP_MAGIC: [u32; 0] = [0u32; 0];

            unsafe { ULP_MAGIC.as_ptr().read_volatile(); }

            use #hal_crate as the_hal;
            #(
                #create_peripheral;
            )*

            main(#(#param_names),*);
        }

        #f
    )
    .into()
}

#[cfg(any(feature = "has-lp-core", feature = "has-ulp-core"))]
pub fn load_lp_code(input: TokenStream) -> TokenStream {
    use std::{fs, path::Path};

    use litrs::StringLit;
    use object::{File, Object, ObjectSection, ObjectSymbol, Section, SectionKind};
    use parse::Error;
    use proc_macro::Span;
    use proc_macro_crate::{crate_name, FoundCrate};
    use syn::{parse, Ident};

    let hal_crate = if cfg!(any(feature = "is-lp-core", feature = "is-ulp-core")) {
        crate_name("esp-lp-hal")
    } else {
        crate_name("esp-hal")
    };

    let hal_crate = if let Ok(FoundCrate::Name(ref name)) = hal_crate {
        let ident = Ident::new(name, Span::call_site().into());
        quote!( #ident )
    } else {
        quote!(crate)
    };

    let first_token = match input.into_iter().next() {
        Some(token) => token,
        None => {
            return Error::new(
                Span::call_site().into(),
                "You need to give the path to an ELF file",
            )
            .to_compile_error()
            .into();
        }
    };
    let arg = match StringLit::try_from(&first_token) {
        Ok(arg) => arg,
        Err(_) => {
            return Error::new(
                Span::call_site().into(),
                "You need to give the path to an ELF file",
            )
            .to_compile_error()
            .into();
        }
    };
    let elf_file = arg.value();

    if !Path::new(elf_file).exists() {
        return Error::new(Span::call_site().into(), "File not found")
            .to_compile_error()
            .into();
    }

    let bin_data = fs::read(elf_file).unwrap();
    let obj_file = File::parse(&*bin_data).unwrap();
    let sections = obj_file.sections();

    let mut sections: Vec<Section> = sections
        .into_iter()
        .filter(|section| {
            matches!(
                section.kind(),
                SectionKind::Text
                    | SectionKind::ReadOnlyData
                    | SectionKind::Data
                    | SectionKind::UninitializedData
            )
        })
        .collect();
    sections.sort_by(|a, b| a.address().partial_cmp(&b.address()).unwrap());

    let mut binary: Vec<u8> = Vec::new();
    let mut last_address = if cfg!(feature = "has-lp-core") {
        0x5000_0000
    } else {
        0x0
    };

    for section in sections {
        if section.address() > last_address {
            let fill = section.address() - last_address;
            binary.extend(std::iter::repeat(0).take(fill as usize));
        }

        binary.extend_from_slice(section.data().unwrap());
        last_address = section.address() + section.size();
    }

    let magic_symbol = obj_file
        .symbols()
        .find(|s| s.name().unwrap().starts_with("__ULP_MAGIC_"));

    let magic_symbol = if let Some(magic_symbol) = magic_symbol {
        magic_symbol.name().unwrap()
    } else {
        return Error::new(
            Span::call_site().into(),
            "Given file doesn't seem to be an LP/ULP core application.",
        )
        .to_compile_error()
        .into();
    };

    let magic_symbol = magic_symbol.trim_start_matches("__ULP_MAGIC_");
    let args: Vec<proc_macro2::TokenStream> = magic_symbol
        .split("$")
        .map(|t| {
            let t = if t.contains("OutputOpenDrain") {
                t.replace("OutputOpenDrain", "LowPowerOutputOpenDrain")
            } else {
                t.replace("Output", "LowPowerOutput")
            };
            let t = t.replace("Input", "LowPowerInput");
            t.parse().unwrap()
        })
        .filter(|v: &proc_macro2::TokenStream| !v.is_empty())
        .collect();

    #[cfg(feature = "has-lp-core")]
    let imports = quote! {
        use #hal_crate::lp_core::LpCore;
        use #hal_crate::lp_core::LpCoreWakeupSource;
        use #hal_crate::gpio::lp_io::LowPowerOutput;
        use #hal_crate::gpio::*;
        use #hal_crate::uart::lp_uart::LpUart;
        use #hal_crate::i2c::lp_i2c::LpI2c;
    };
    #[cfg(feature = "has-ulp-core")]
    let imports = quote! {
        use #hal_crate::ulp_core::UlpCore as LpCore;
        use #hal_crate::ulp_core::UlpCoreWakeupSource as LpCoreWakeupSource;
        use #hal_crate::gpio::*;
    };

    #[cfg(feature = "has-lp-core")]
    let rtc_code_start = quote! { _rtc_fast_data_start };
    #[cfg(feature = "has-ulp-core")]
    let rtc_code_start = quote! { _rtc_slow_data_start };

    quote! {
        {
            #imports

            struct LpCoreCode {}

            static LP_CODE: &[u8] = &[#(#binary),*];

            extern "C" {
                static #rtc_code_start: u32;
            }

            unsafe {
                core::ptr::copy_nonoverlapping(LP_CODE as *const _ as *const u8, &#rtc_code_start as *const u32 as *mut u8, LP_CODE.len());
            }

            impl LpCoreCode {
                pub fn run(
                    &self,
                    lp_core: &mut LpCore,
                    wakeup_source: LpCoreWakeupSource,
                    #(_: #args),*
                ) {
                    lp_core.run(wakeup_source);
                }
            }

            LpCoreCode {}
        }
    }
    .into()
}
