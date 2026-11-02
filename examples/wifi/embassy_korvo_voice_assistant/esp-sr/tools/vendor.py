#!/usr/bin/env python3
"""Prepares the ESP-SR blobs that this crate links, from an ESP-SR release.

    tools/vendor.py <espressif__esp-sr component directory>

Needs Espressif's RISC-V binutils (riscv32-esp-elf-ar, -objcopy, -ranlib) in PATH, only here, not
for building the crate. For each library that the MultiNet speech recognizer needs, it:

- Marks the Espressif relocations (R_RISCV_VENDOR and R_RISCV_ESP_LP_OFFSET_12) as R_RISCV_NONE.
  They only appear in hand-written assembly, for the offsets of hardware loops to labels in the
  same section. The assembler already encodes those offsets, GNU ld does not change them, and
  LLD rejects the relocations.
- Renames the imports listed in `renames.txt`, which `src/sys.rs` implements.

It also copies the English MultiNet7 model and ESP-SR's license.
"""
import pathlib, re, shutil, struct, subprocess, sys, tempfile

VERSION = "2.6.0"
TARGET = "esp32s31"
LIBRARIES = ["multinet", "flite_g2p", "fst", "dl_lib", "c_speech_features", "hufzip"]
MODEL_FILES = {
    "mn7_en": ["_MODEL_INFO_", "mn7_data", "mn7_index", "vocab"],
    "fst": ["commands_en.txt"],
}
R_RISCV_VENDOR, R_RISCV_ESP_LP_OFFSET_12 = 191, 193
SHT_RELA = 4

def neutralize_vendor_relocations(path):
    data = bytearray(path.read_bytes())
    assert data[:4] == b"\x7fELF" and data[4] == 1 and data[5] == 1, f"{path} is not ELF32 LE"
    shoff, = struct.unpack_from("<I", data, 0x20)
    shentsize, shnum = struct.unpack_from("<HH", data, 0x2E)
    patched = 0
    for i in range(shnum):
        _, sh_type, _, _, offset, size, _, _, _, entsize = struct.unpack_from(
            "<10I", data, shoff + i * shentsize)
        if sh_type != SHT_RELA:
            continue
        for entry in range(offset, offset + size, entsize):
            r_info_type = entry + 4  # The low byte of r_info is the relocation type.
            if data[r_info_type] in (R_RISCV_VENDOR, R_RISCV_ESP_LP_OFFSET_12):
                data[r_info_type] = 0
                patched += 1
    path.write_bytes(data)
    return patched

def run(*args, cwd=None):
    return subprocess.run(args, cwd=cwd, check=True, capture_output=True, text=True).stdout

def main():
    component = pathlib.Path(sys.argv[1])
    crate = pathlib.Path(__file__).resolve().parent.parent
    manifest = (component / "idf_component.yml").read_text()
    # The Git repository quotes the version, the component registry's packages do not.
    version = re.search(r'^version:\s*"?([^"\s]+)"?\s*$', manifest, re.MULTILINE)
    assert version and version.group(1) == VERSION, f"Expected ESP-SR {VERSION}"

    out = crate / "lib" / TARGET
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    renames = crate / "tools" / "renames.txt"
    for library in LIBRARIES:
        archive = component / "lib" / TARGET / f"lib{library}.a"
        members = run("riscv32-esp-elf-ar", "t", str(archive)).split()
        with tempfile.TemporaryDirectory() as work:
            run("riscv32-esp-elf-ar", "x", str(archive), cwd=work)
            patched = 0
            for member in members:
                patched += neutralize_vendor_relocations(pathlib.Path(work) / member)
                run("riscv32-esp-elf-objcopy", "--strip-debug", f"--redefine-syms={renames}", member, cwd=work)
            target = out / f"lib{library}.a"
            run("riscv32-esp-elf-ar", "rcD", str(target), *members, cwd=work)
            run("riscv32-esp-elf-ranlib", "-D", str(target))
        print(f"lib{library}.a: {len(members)} members, {patched} vendor relocations neutralized")

    model = crate / "model"
    shutil.rmtree(model, ignore_errors=True)
    for name, files in MODEL_FILES.items():
        (model / name).mkdir(parents=True)
        for file in files:
            shutil.copy(component / "model" / "multinet_model" / name / file, model / name / file)
    shutil.copy(component / "LICENSE", crate / "LICENSE-ESP-SR")
    print(f"Vendored ESP-SR {VERSION} for {TARGET}")

if __name__ == "__main__":
    main()
