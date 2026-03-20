/* # EXCEPTION HANDLERS DESCRIBED IN THE STANDARD RISC-V ISA
   
   If the `no-exceptions` feature is DISABLED, this file will be included in link.x.in.
   If the `no-exceptions` feature is ENABLED, this file will be ignored.
*/

/* It is possible to define a special handler for each exception type.
   By default, all exceptions are handled by ExceptionHandler. However,
   users can override these alias by defining the symbol themselves */
PROVIDE(InstructionMisaligned = ExceptionHandler);
PROVIDE(InstructionFault = ExceptionHandler);
PROVIDE(IllegalInstruction = ExceptionHandler);
PROVIDE(Breakpoint = ExceptionHandler);
PROVIDE(LoadMisaligned = ExceptionHandler);
PROVIDE(LoadFault = ExceptionHandler);
PROVIDE(StoreMisaligned = ExceptionHandler);
PROVIDE(StoreFault = ExceptionHandler);
PROVIDE(UserEnvCall = ExceptionHandler);
PROVIDE(SupervisorEnvCall = ExceptionHandler);
PROVIDE(MachineEnvCall = ExceptionHandler);
PROVIDE(InstructionPageFault = ExceptionHandler);
PROVIDE(LoadPageFault = ExceptionHandler);
PROVIDE(StorePageFault = ExceptionHandler);
