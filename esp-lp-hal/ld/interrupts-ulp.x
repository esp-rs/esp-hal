/* # CORE INTERRUPT HANDLERS DESCRIBED IN THE STANDARD RISC-V ISA
   
   If the `no-interrupts` feature is DISABLED, this file will be included in link.x.in.
   If the `no-interrupts` feature is ENABLED, this file will be ignored.
*/

/* It is possible to define a special handler for each interrupt type.
   By default, all interrupts are handled by DefaultHandler. However, users can
   override these alias by defining the symbol themselves */
/* PROVIDE(SupervisorSoft = DefaultHandler); */
/* PROVIDE(MachineSoft = DefaultHandler); */
/* PROVIDE(SupervisorTimer = DefaultHandler); */
/* PROVIDE(MachineTimer = DefaultHandler); */
/* PROVIDE(SupervisorExternal = DefaultHandler); */
/* PROVIDE(MachineExternal = DefaultHandler); */

PROVIDE(Timer = DefaultHandler);
PROVIDE(Peripheral = DefaultHandler);