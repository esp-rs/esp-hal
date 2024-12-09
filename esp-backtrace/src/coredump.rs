const HEADER_SIZE: u32 = 0x34;
const PROGRAM_HEADER_SIZE: u32 = 0x20;
const REG_INFO_HEADER_SIZE: usize = 20;

enum SegmentType {
    Load = 1,
    Note = 4,
}

#[cfg(target_arch = "xtensa")]
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Registers {
    pub pc: u32,
    pub ps: u32,
    pub lbeg: u32,
    pub lend: u32,
    pub lcount: u32,
    pub sar: u32,
    pub windowstart: u32,
    pub windowbase: u32,
    pub reserved: [u32; 8 + 48],
    pub ar: [u32; 64],
}

#[cfg(target_arch = "xtensa")]
impl Default for Registers {
    fn default() -> Self {
        Self {
            pc: Default::default(),
            ps: Default::default(),
            lbeg: Default::default(),
            lend: Default::default(),
            lcount: Default::default(),
            sar: Default::default(),
            windowstart: Default::default(),
            windowbase: Default::default(),
            reserved: [0u32; 8 + 48],
            ar: [0u32; 64],
        }
    }
}

#[cfg(target_arch = "riscv32")]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Registers {
    pub pc: u32,
    pub x1: u32,
    pub x2: u32,
    pub x3: u32,
    pub x4: u32,
    pub x5: u32,
    pub x6: u32,
    pub x7: u32,
    pub x8: u32,
    pub x9: u32,
    pub x10: u32,
    pub x11: u32,
    pub x12: u32,
    pub x13: u32,
    pub x14: u32,
    pub x15: u32,
    pub x16: u32,
    pub x17: u32,
    pub x18: u32,
    pub x19: u32,
    pub x20: u32,
    pub x21: u32,
    pub x22: u32,
    pub x23: u32,
    pub x24: u32,
    pub x25: u32,
    pub x26: u32,
    pub x27: u32,
    pub x28: u32,
    pub x29: u32,
    pub x30: u32,
    pub x31: u32,
}

#[cfg(target_arch = "riscv32")]
impl Default for Registers {
    fn default() -> Self {
        Self {
            pc: Default::default(),
            x1: Default::default(),
            x2: Default::default(),
            x3: Default::default(),
            x4: Default::default(),
            x5: Default::default(),
            x6: Default::default(),
            x7: Default::default(),
            x8: Default::default(),
            x9: Default::default(),
            x10: Default::default(),
            x11: Default::default(),
            x12: Default::default(),
            x13: Default::default(),
            x14: Default::default(),
            x15: Default::default(),
            x16: Default::default(),
            x17: Default::default(),
            x18: Default::default(),
            x19: Default::default(),
            x20: Default::default(),
            x21: Default::default(),
            x22: Default::default(),
            x23: Default::default(),
            x24: Default::default(),
            x25: Default::default(),
            x26: Default::default(),
            x27: Default::default(),
            x28: Default::default(),
            x29: Default::default(),
            x30: Default::default(),
            x31: Default::default(),
        }
    }
}

pub struct Memory<'a> {
    pub start: u32,
    pub slice: &'a [u8],
}

#[cfg(target_arch = "xtensa")]
#[repr(C, packed)]
struct RegisterInfo {
    // PR status
    si_signo: u32,
    si_code: u32,
    si_errno: u32,
    pr_cursig: u16,
    pr_pad0: u16,
    pr_sigpend: u32,
    pr_sighold: u32,
    pr_pid: u32,
    pr_ppid: u32,
    pr_pgrp: u32,
    pr_sid: u32,
    pr_utime: u64,
    pr_stime: u64,
    pr_cutime: u64,
    pr_cstime: u64,

    registers: Registers,

    reserved_: u32,
}

#[cfg(target_arch = "xtensa")]
impl Default for RegisterInfo {
    fn default() -> Self {
        Self {
            si_signo: 0,
            si_code: 0,
            si_errno: 0,
            pr_cursig: 0,
            pr_pad0: 0,
            pr_sigpend: 0,
            pr_sighold: 0,
            pr_pid: 0,
            pr_ppid: 0,
            pr_pgrp: 0,
            pr_sid: 0,
            pr_utime: 0,
            pr_stime: 0,
            pr_cutime: 0,
            pr_cstime: 0,

            registers: Registers::default(),

            reserved_: 0,
        }
    }
}

// see ESP-IDF for how this looks like for Xtensa
// see components\espcoredump\src\port\xtensa\core_dump_port.c for
#[cfg(target_arch = "riscv32")]
#[repr(C)]
struct RegisterInfo {
    padding1: [u8; 12],
    signal: u16,
    padding2: [u8; 10],
    pid: u32,
    padding3: [u8; 44],

    registers: Registers,

    padding4: [u8; 4],
}

#[cfg(target_arch = "riscv32")]
impl Default for RegisterInfo {
    fn default() -> Self {
        Self {
            padding1: Default::default(),
            signal: Default::default(),
            padding2: Default::default(),
            pid: Default::default(),
            padding3: [0u8; 72 - 24 - 4],
            registers: Default::default(),
            padding4: Default::default(),
        }
    }
}

pub fn dump<W: embedded_io::Write>(
    writer: &mut W,
    regs: &Registers,
    mem: Memory,
) -> Result<(), W::Error> {
    let mut writer = CoreDumpWriter::new(writer);

    let reg_info: RegisterInfo = {
        let mut reg_info = RegisterInfo::default();
        reg_info.registers = *regs;
        reg_info
    };

    writer.elf_header()?;

    // header for the memory region
    writer.write_program_header(
        SegmentType::Load,
        HEADER_SIZE + 2 * PROGRAM_HEADER_SIZE, // we have 2 program header entries
        mem.start,
        mem.slice.len() as u32,
        6,
        0,
    )?;

    // header for the register info
    writer.write_program_header(
        SegmentType::Note,
        // we have 2 program header entries and first data is the memory block
        HEADER_SIZE + 2 * PROGRAM_HEADER_SIZE + mem.slice.len() as u32,
        0,
        (core::mem::size_of::<RegisterInfo>() + REG_INFO_HEADER_SIZE) as u32,
        6,
        0,
    )?;

    // write memory contents
    writer.writer.write(mem.slice)?;

    // write register info
    let mut reg_info_bytes = [0x0u8; core::mem::size_of::<RegisterInfo>() + REG_INFO_HEADER_SIZE];
    unsafe {
        (&reg_info as *const RegisterInfo)
            .copy_to(reg_info_bytes.as_mut_ptr() as *mut RegisterInfo, 1);
    }

    // reg-info-header

    // always like this so we can hardcode it
    // slightly different for Xtensa b.c. the length of the register info (0xcc) is
    // different
    #[cfg(target_arch = "riscv32")]
    writer.writer.write(&[
        0x08, 0x00, 0x00, 0x00, 0xcc, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x43, 0x4f, 0x52,
        0x45, 0x00, 0x00, 0x00, 0x00,
    ])?;

    #[cfg(target_arch = "xtensa")]
    writer.writer.write(&[
        0x08, 0x00, 0x00, 0x00, 0x4c, 0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x43, 0x4f, 0x52,
        0x45, 0x00, 0x00, 0x00, 0x00,
    ])?;

    writer.writer.write(&reg_info_bytes)?;

    Ok(())
}

struct CoreDumpWriter<'a, W: embedded_io::Write> {
    writer: &'a mut W,
}

impl<'a, W: embedded_io::Write> CoreDumpWriter<'a, W> {
    pub fn new(writer: &'a mut W) -> Self {
        Self { writer }
    }

    fn elf_header(&mut self) -> Result<(), W::Error> {
        let _ = self.writer.write(&[0x7f, b'E', b'L', b'F'])?;
        let _ = self.writer.write(&[0x01])?; // 32 bit
        let _ = self.writer.write(&[0x01])?; // little endian
        let _ = self.writer.write(&[0x01])?; // version
        let _ = self.writer.write(&[0x00])?; // ABI
        let _ = self.writer.write(&[0x00])?; // ABI version
        let _ = self
            .writer
            .write(&[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])?; // padding
        let _ = self.writer.write(&[0x04, 0x00])?; // CORE file

        #[cfg(target_arch = "riscv32")]
        let _ = self.writer.write(&[0xf3, 0x00])?; // machine (RISCV)

        #[cfg(target_arch = "xtensa")]
        let _ = self.writer.write(&[0x5e, 0x00])?; // machine (Xtensa)

        let _ = self.writer.write(&[0x01, 0x00, 0x00, 0x00])?; // version
        let _ = self.writer.write(&[0x00, 0x00, 0x00, 0x00])?; // entry
        let _ = self.writer.write(&[0x34, 0x00, 0x00, 0x00])?; // start of program header
        let _ = self.writer.write(&[0x00, 0x00, 0x00, 0x00])?; // start of section header
        let _ = self.writer.write(&[0x00, 0x00, 0x00, 0x00])?; // flags
        let _ = self.writer.write(&[0x34, 0x00])?; // ehsize
        let _ = self.writer.write(&[0x20, 0x00])?; // size of a program header table entry.
        let _ = self.writer.write(&[0x02, 0x00])?; // number of entries in the program header table !!!! here 2: one memory block
                                                   // and the registers as a note
        let _ = self.writer.write(&[0x28, 0x00])?; // size of a section header table entry.
        let _ = self.writer.write(&[0x00, 0x00])?; // number of section headers
        let _ = self.writer.write(&[0x00, 0x00])?; // index of the section header table
                                                   // entry that contains the section
                                                   // name
        Ok(())
    }

    fn write_program_header(
        &mut self,
        stype: SegmentType,
        offset: u32,
        addr: u32,
        size: u32,
        flags: u32,
        align: u32,
    ) -> Result<(), W::Error> {
        let _ = self.writer.write(&(stype as u32).to_le_bytes())?; // type
        let _ = self.writer.write(&offset.to_le_bytes())?; // offset in file
        let _ = self.writer.write(&addr.to_le_bytes())?; // vaddr
        let _ = self.writer.write(&addr.to_le_bytes())?; // paddr
        let _ = self.writer.write(&size.to_le_bytes())?; // file size
        let _ = self.writer.write(&size.to_le_bytes())?; // memory size
        let _ = self.writer.write(&flags.to_le_bytes())?; // flags
        let _ = self.writer.write(&align.to_le_bytes())?; // align
        Ok(())
    }
}

#[derive(Debug)]
pub struct DumpWriter {}

impl embedded_io::Error for DumpWriter {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl embedded_io::ErrorType for DumpWriter {
    type Error = core::convert::Infallible;
}

impl embedded_io::Write for DumpWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for _b in buf.iter() {
            #[cfg(feature = "println")]
            esp_println::print!("{:02x}", _b);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
