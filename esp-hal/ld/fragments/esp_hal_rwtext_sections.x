/* Output sections in RWTEXT only. The bootloader support crate includes this file inside `SECTIONS`,
   in front of the output section that holds `*(.rwtext.*)`, so these sections get their input
   sections first. */
