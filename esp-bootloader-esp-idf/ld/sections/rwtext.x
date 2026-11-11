INCLUDE "trap.x"

INCLUDE "esp_hal_rwtext_sections.x"

.rwtext : ALIGN(4)
{
  . = ALIGN (4);

  /* Keep each object's literal pool adjacent to its code (see comment in `text.x`). */
  *(.rwtext.literal .rwtext .rwtext.literal.* .rwtext.*)

  INCLUDE "esp_hal_rwtext.x"

  . = ALIGN(4);
} > RWTEXT

.rwtext.wifi :
{
  . = ALIGN(4);
  INCLUDE "esp_hal_rwtext_radio.x"
  . = ALIGN(4);

  _rwtext_end = .;
  _rwtext_len = ABSOLUTE(. - ORIGIN(RWTEXT));
} > RWTEXT
