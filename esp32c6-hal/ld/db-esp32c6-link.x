INCLUDE memory.x

SECTIONS
{
  .header : AT(0)
  {
    LONG(0xaedb041d)
    LONG(0xaedb041d)
  } > ROM
}

_stext = ORIGIN(ROM) + 8;

INCLUDE riscv-link.x
