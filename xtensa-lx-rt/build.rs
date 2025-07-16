use std::{
    collections::{HashMap, HashSet},
    env,
    fs,
    path::{Path, PathBuf},
};

type Result<T> = ::std::result::Result<T, Box<dyn std::error::Error>>;

/// The chips which are present in the xtensa-overlays repository
///
/// When `.to_string()` is called on a variant, the resulting string is the path
/// to the chip's corresponding directory.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Chip {
    Esp32,
    Esp32s2,
    Esp32s3,
}

impl Chip {
    const TARGET_TO_CHIP: &'static [(&'static str, Chip)] = &[
        ("xtensa-esp32-none-elf", Chip::Esp32),
        ("xtensa-esp32s2-none-elf", Chip::Esp32s2),
        ("xtensa-esp32s3-none-elf", Chip::Esp32s3),
    ];

    fn config(&self) -> HashMap<&'static str, Value> {
        let mut config = std::collections::HashMap::new();

        match self {
            Chip::Esp32 => include!("config/esp32.rs"),
            Chip::Esp32s2 => include!("config/esp32s2.rs"),
            Chip::Esp32s3 => include!("config/esp32s3.rs"),
        }

        config
    }
}

/// The valid interrupt types declared in the `core-isa.h` headers
#[derive(Debug, Clone, Copy, PartialEq)]
enum InterruptType {
    ExternEdge,
    ExternLevel,
    Nmi,
    Profiling,
    Software,
    Timer,
    TimerUnconfigured,
}

/// The allowable value types for definitions
#[derive(Debug, Clone, PartialEq)]
enum Value {
    Integer(i64),
    Interrupt(InterruptType),
    String(&'static str),
}

impl Value {
    #[inline]
    fn as_integer(&self) -> Option<i64> {
        match self {
            Self::Integer(inner) => Some(*inner),
            _ => None,
        }
    }

    #[inline]
    fn is_integer(&self) -> bool {
        matches!(self, Self::Integer(_))
    }
}

fn main() -> Result<()> {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // Put the linker script somewhere the linker can find it
    println!("cargo:rustc-link-search={}", out.display());

    fs::write(out.join("link.x"), include_bytes!("xtensa.in.x"))?;

    handle_esp32()?;

    // Only re-run the build script when xtensa.in.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=xtensa.in.x");

    Ok(())
}

fn handle_esp32() -> Result<()> {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    let rustflags = env::var_os("CARGO_ENCODED_RUSTFLAGS")
        .unwrap()
        .into_string()
        .unwrap();

    let mut features_to_disable = HashSet::<&'static str>::new();

    // Users can pass -Ctarget-feature to the compiler multiple times, so we have to
    // handle that
    let target_flags = rustflags
        .split(0x1f as char)
        .filter(|s| s.starts_with("target-feature="))
        .filter_map(|s| s.strip_prefix("target-feature="));
    for tf in target_flags {
        tf.split(',')
            .map(|s| s.trim())
            .filter_map(|s| s.strip_prefix('-'))
            .filter_map(rustc_feature_to_xchal_have)
            .for_each(|s| {
                features_to_disable.insert(s);
            })
    }

    // Do not check target when building documentation, but do check for doc-tests
    let chip = if std::env::var("RUSTDOCFLAGS").is_err() || std::env::var("ESP_HAL_DOCTEST").is_ok()
    {
        // Based on the build target, determine which chip to use.
        let target = std::env::var("TARGET");
        let target = target.as_deref().unwrap_or("unspecified target");
        let Some(chip) = Chip::TARGET_TO_CHIP
            .iter()
            .copied()
            .find_map(|(t, chip)| (t == target).then_some(chip))
        else {
            panic!(
                "Unsupported target: {target}. Expected one of: {}",
                Chip::TARGET_TO_CHIP
                    .iter()
                    .map(|(t, _)| t.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        };
        chip
    } else {
        // For documentation purposes, we use ESP32
        Chip::Esp32
    };

    let isa_config = chip.config();

    inject_cfgs(&isa_config, &features_to_disable);
    inject_cpu_cfgs(&isa_config);
    generate_exception_x(out, &isa_config)?;
    generate_interrupt_level_masks(out, &isa_config)?;

    Ok(())
}

fn generate_interrupt_level_masks(out: &Path, isa_config: &HashMap<&str, Value>) -> Result<()> {
    let exception_source_template = include_str!("interrupt_level_masks.rs.template");

    let mut masks = exception_source_template.to_string();
    for mask in [
        "XCHAL_INTLEVEL1_MASK",
        "XCHAL_INTLEVEL2_MASK",
        "XCHAL_INTLEVEL3_MASK",
        "XCHAL_INTLEVEL4_MASK",
        "XCHAL_INTLEVEL5_MASK",
        "XCHAL_INTLEVEL6_MASK",
        "XCHAL_INTLEVEL7_MASK",
    ] {
        masks = masks.replace(
            &format!("{{{{ {mask} }}}}"),
            &isa_config
                .get(mask)
                .unwrap()
                .as_integer()
                .unwrap()
                .to_string(),
        );
    }

    fs::write(out.join("interrupt_level_masks.rs"), masks)?;

    Ok(())
}

fn generate_exception_x(out: &Path, _isa_config: &HashMap<&str, Value>) -> Result<()> {
    let exception_source_template = include_str!("exception-esp32.x.template");

    fs::write(out.join("exception.x"), exception_source_template)?;

    Ok(())
}

fn inject_cfgs(isa_config: &HashMap<&str, Value>, disabled_features: &HashSet<&str>) {
    for (key, value) in isa_config {
        if key.starts_with("XCHAL_HAVE")
            && value.as_integer().unwrap_or(0) != 0
            && !disabled_features.contains(key)
        {
            println!("cargo:rustc-cfg={key}");
        }
    }
}

fn inject_cpu_cfgs(isa_config: &HashMap<&str, Value>) {
    for (key, value) in isa_config {
        if (key.starts_with("XCHAL_TIMER")
            || key.starts_with("XCHAL_PROFILING")
            || key.starts_with("XCHAL_NMI"))
            && value.is_integer()
        {
            let s = key
                .trim_start_matches("XCHAL_")
                .trim_end_matches("_INTERRUPT");
            println!("cargo:rustc-cfg=XCHAL_HAVE_{s}");
        }
    }
    if let Some(value) = isa_config
        .get("XCHAL_INTTYPE_MASK_SOFTWARE")
        .and_then(|v| v.as_integer())
    {
        for i in 0..value.count_ones() {
            println!("cargo:rustc-cfg=XCHAL_HAVE_SOFTWARE{i}");
        }
    }
}

fn rustc_feature_to_xchal_have(s: &str) -> Option<&str> {
    // List of rustc features taken from here:
    // https://github.com/esp-rs/rust/blob/84ecb3f010525cb1b2e7d4da306099c2eaa3e6cd/compiler/rustc_codegen_ssa/src/target_features.rs#L278
    // unlikely to change
    Some(match s {
        "fp" => "XCHAL_HAVE_FP",
        "windowed" => "XCHAL_HAVE_WINDOWED",
        "bool" => "XCHAL_HAVE_BOOLEANS",
        "loop" => "XCHAL_HAVE_LOOPS",
        "sext" => "XCHAL_HAVE_SEXT",
        "nsa" => "XCHAL_HAVE_NSA",
        "mul32" => "XCHAL_HAVE_MUL32",
        "mul32high" => "XCHAL_HAVE_MUL32_HIGH",
        "div32" => "XCHAL_HAVE_DIV32",
        "mac16" => "XCHAL_HAVE_MAC16",
        "dfpaccel" => "XCHAL_HAVE_DFP",
        "s32c1i" => "XCHAL_HAVE_S32C1I",
        "threadptr" => "XCHAL_HAVE_THREADPTR",
        "extendedl32r" => "XCHAL_HAVE_ABSOLUTE_LITERALS",
        "debug" => "XCHAL_HAVE_DEBUG",
        "exception" => "XCHAL_HAVE_EXCEPTIONS",
        "highpriinterrupts" => "XCHAL_HAVE_HIGHPRI_INTERRUPTS",
        "coprocessor" => "XCHAL_HAVE_CP",
        "interrupt" => "XCHAL_HAVE_INTERRUPTS",
        "rvector" => "XCHAL_HAVE_VECTOR_SELECT",
        "prid" => "XCHAL_HAVE_PRID",
        "regprotect" => "XCHAL_HAVE_MIMIC_CACHEATTR",
        "miscsr" => return None,   // XCHAL_NUM_MISC_REGS
        "timerint" => return None, // XCHAL_NUM_TIMERS
        "atomctl" => return None,
        "memctl" => return None,
        _ => return None,
    })
}
