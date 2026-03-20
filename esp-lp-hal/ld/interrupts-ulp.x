/* # CORE INTERRUPT HANDLERS DESCRIBED IN THE STANDARD RISC-V ISA
   
   If the `no-interrupts` feature is DISABLED, this file will be included in link.x.in.
   If the `no-interrupts` feature is ENABLED, this file will be ignored.
*/

/* It is possible to define a special handler for each interrupt type.
   By default, all interrupts are handled by DefaultHandler. However, users can
   override these alias by defining the symbol themselves */
PROVIDE(SupervisorSoft = DefaultHandler);
PROVIDE(MachineSoft = DefaultHandler);
PROVIDE(SupervisorTimer = DefaultHandler);
PROVIDE(MachineTimer = DefaultHandler);
PROVIDE(SupervisorExternal = DefaultHandler);
PROVIDE(MachineExternal = DefaultHandler);

/* When vectored trap mode is enabled, each interrupt source must implement its own
   trap entry point. By default, all interrupts start in _DefaultHandler_trap.
   However, users can override these alias by defining the symbol themselves */
PROVIDE(_start_SupervisorSoft_trap = _start_DefaultHandler_trap);
PROVIDE(_start_MachineSoft_trap = _start_DefaultHandler_trap);
PROVIDE(_start_SupervisorTimer_trap = _start_DefaultHandler_trap);
PROVIDE(_start_MachineTimer_trap = _start_DefaultHandler_trap);
PROVIDE(_start_SupervisorExternal_trap = _start_DefaultHandler_trap);
PROVIDE(_start_MachineExternal_trap = _start_DefaultHandler_trap);
