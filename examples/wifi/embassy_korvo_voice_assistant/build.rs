//! Builds the C sources of ESP-SR's MultiNet speech recognizer and links them with the prebuilt
//! ESP-SR libraries and picolibc, the C library the prebuilt libraries were built with.
//!
//! `ESP_SR_COMPONENTS` is a directory with the ESP-IDF components `espressif__esp-sr`,
//! `espressif__dl_fft` and `espressif__cjson`, as the ESP-IDF component manager downloads them.

use std::{
    env,
    fs,
    io::Write,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

const COMPILER: &str = "riscv32-esp-elf-gcc";
const ARCHIVER: &str = "riscv32-esp-elf-ar";
const RANLIB: &str = "riscv32-esp-elf-ranlib";
/// The ESP32-S31 instruction set without Espressif's extensions, which the C sources do not use.
const ARCH: [&str; 2] = [
    "-march=rv32imafc_zicsr_zifencei_zaamo_zalrsc",
    "-mabi=ilp32f",
];

/// The prebuilt libraries MultiNet needs, in `lib/esp32s31` of ESP-SR.
const SR_LIBRARIES: [&str; 6] = [
    "multinet",
    "flite_g2p",
    "fst",
    "dl_lib",
    "c_speech_features",
    "hufzip",
];

/// Members left out of the merged archive, because other members define the same symbols:
/// picolibc's `printf` family, which the Wi-Fi libraries also define, and an ESP-SR object whose
/// three functions `dl_esp32p4_sr_s8_unaligned_conv2d.S.obj` also contains.
const LEFT_OUT_MEMBERS: [&str; 6] = [
    "libc_tinystdio_printf.c.o",
    "libc_tinystdio_sprintf.c.o",
    "libc_tinystdio_snprintf.c.o",
    "libc_tinystdio_vprintf.c.o",
    "libc_tinystdio_vsnprintf.c.o",
    "dl_esp32p4_sr_common.S.obj",
];

/// Wi-Fi libraries that define symbols other Wi-Fi libraries use, but come first on the linker
/// command line. GNU ld, unlike LLD, only looks for symbols in the libraries that follow.
const LATE_WIFI_LIBRARIES: [&str; 3] = ["core", "espnow", "mesh"];

fn main() {
    println!("cargo:rerun-if-env-changed=ESP_SR_COMPONENTS");
    println!("cargo:rerun-if-changed=c");
    println!("cargo:rerun-if-changed=picolibc_tls.x");
    let components = PathBuf::from(env::var("ESP_SR_COMPONENTS").unwrap());
    let esp_sr = component(&components, "espressif__esp-sr");
    let dl_fft = component(&components, "espressif__dl_fft");
    let cjson = component(&components, "espressif__cjson");

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    cc::Build::new()
        .compiler(COMPILER)
        .archiver(ARCHIVER)
        .ranlib(RANLIB)
        .flags(ARCH)
        .flag("--specs=picolibc.specs")
        .warnings(false)
        .cargo_metadata(false)
        // Also turns on the grapheme-to-phoneme conversion of English commands.
        .define("CONFIG_SR_MN_EN_MULTINET7_QUANT", "1")
        // The defaults of the ESP-IDF component.
        .define("CJSON_NESTING_LIMIT", "1000")
        .define("CJSON_CIRCULAR_LIMIT", "10000")
        .include("c/include")
        .include(esp_sr.join("include/esp32s31"))
        .include(esp_sr.join("src/include"))
        .include(&dl_fft)
        .include(dl_fft.join("base"))
        .include(dl_fft.join("base/isa"))
        .include(cjson.join("cJSON"))
        .file("c/stdio.c")
        .file(esp_sr.join("src/esp_mn_speech_commands.c"))
        .file(cjson.join("cJSON/cJSON.c"))
        // dl_fft without its assembly, which needs ESP-IDF.
        .files(c_files(&dl_fft))
        .files(c_files(&dl_fft.join("base")))
        .compile("esp_sr_sources");

    // One archive with the sources, the prebuilt libraries and picolibc, which need each other. GNU
    // ld searches one archive until it finds no more symbols, but each archive only once. The
    // archive comes before the Wi-Fi libraries, so picolibc's stubs of `gettimeofday` and similar
    // do not replace the functions esp-radio provides to them. The sources come first, so their
    // `stdout` replaces picolibc's.
    let sysroot = compiler_output(&["-print-sysroot"]);
    let multilib = compiler_output(&["-print-multi-directory"]);
    let picolibc = Path::new(&sysroot)
        .join("../picolibc/riscv32-esp-elf/lib")
        .join(multilib);
    let libraries = [out_dir.join("libesp_sr_sources.a")]
        .into_iter()
        .chain(SR_LIBRARIES.map(|name| esp_sr.join(format!("lib/esp32s31/lib{name}.a"))))
        .chain([picolibc.join("libc.a")]);
    merge_archives(&out_dir.join("libesp_sr.a"), libraries);
    println!("cargo:rustc-link-search=native={}", out_dir.display());
    println!("cargo:rustc-link-lib=static=esp_sr");
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-arg=-T{manifest_dir}/picolibc_tls.x");

    println!("cargo:rustc-link-arg=--start-group");
    for library in LATE_WIFI_LIBRARIES {
        println!("cargo:rustc-link-arg=-l{library}");
    }
    println!("cargo:rustc-link-arg=--end-group");
}

fn component(components: &Path, name: &str) -> PathBuf {
    let path = components.join(name);
    assert!(
        path.is_dir(),
        "{} does not exist. Set ESP_SR_COMPONENTS to a directory with the ESP-IDF components \
         espressif__esp-sr, espressif__dl_fft and espressif__cjson.",
        path.display()
    );
    path
}

fn c_files(dir: &Path) -> impl Iterator<Item = PathBuf> {
    fs::read_dir(dir)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension().is_some_and(|extension| extension == "c"))
}

/// Runs the compiler for the target ABI and returns what it prints.
fn compiler_output(args: &[&str]) -> String {
    let output = Command::new(COMPILER)
        .args(ARCH)
        .args(args)
        .output()
        .unwrap_or_else(|error| panic!("Failed to run {COMPILER}, is it in PATH? {error}"));
    assert!(output.status.success(), "{COMPILER} {args:?} failed");
    String::from_utf8(output.stdout).unwrap().trim().into()
}

/// Writes the members of `libraries` to one archive, in order, except [`LEFT_OUT_MEMBERS`].
fn merge_archives(archive: &Path, libraries: impl Iterator<Item = PathBuf>) {
    let mut script = format!("CREATE {}\n", archive.display());
    for library in libraries {
        script += &format!("ADDLIB {}\n", library.display());
    }
    for member in LEFT_OUT_MEMBERS {
        script += &format!("DELETE {member}\n");
    }
    script += "SAVE\nEND\n";

    let mut ar = Command::new(ARCHIVER)
        .arg("-M")
        .stdin(Stdio::piped())
        .spawn()
        .unwrap();
    ar.stdin
        .take()
        .unwrap()
        .write_all(script.as_bytes())
        .unwrap();
    assert!(
        ar.wait().unwrap().success(),
        "Failed to create {}",
        archive.display()
    );
}
