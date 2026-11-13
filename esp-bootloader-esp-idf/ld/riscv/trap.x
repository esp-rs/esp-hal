.trap : ALIGN(4)
{
  _trap_section_origin = .;
  KEEP(*(.trap));
  *(.trap.*);

  *(.rwtext.interrupt .rwtext.interrupt.*)

  /* PMP write-protects the code from here. The trap section stays writable. */
  _rwtext_start = .;
} > RWTEXT
