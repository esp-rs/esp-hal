/* The Espressif blobs (esp-wifi-sys / esp-phy / BT) are GCC-compiled and
   emit per-function `.literal.<f>` paired with `.text.<f>` within each
   object file. Pull each archive's sections in as a single block using
   the combined pattern: both GNU ld and LLD preserve input-file order
   within a single `*(...)` group, which keeps every function's literal
   pool within L32R reach of its own code regardless of total image size.
   Without these per-archive selectors, a catch-all would bunch all
   literals at the start and push wifi text past the 256 KB L32R window. */
*libpp.a:(.literal .text .literal.* .text.*)
*libnet80211.a:(.literal .text .literal.* .text.*)
*libwpa_supplicant.a:(.literal .text .literal.* .text.*)
*libmesh.a:(.literal .text .literal.* .text.*)
*libsmartconfig.a:(.literal .text .literal.* .text.*)
*libphy.a:(.literal .text .literal.* .text.*)
*libcoexist.a:(.literal .text .literal.* .text.*)
*libcore.a:(.literal .text .literal.* .text.*)
*libregulatory.a:(.literal .text .literal.* .text.*)
*libespnow.a:(.literal .text .literal.* .text.*)
*libwapi.a:(.literal .text .literal.* .text.*)
*libprintf.a:(.literal .text .literal.* .text.*)
*libbtdm_app.a:(.literal .text .literal.* .text.*)
*libbtbb.a:(.literal .text .literal.* .text.*)
