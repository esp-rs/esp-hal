.trap : ALIGN(4)
{
  _trap_section_origin = .;
  KEEP(*(.trap));
  *(.trap.*);

  *(.rwtext.interrupt .rwtext.interrupt.*)
} > RWTEXT
