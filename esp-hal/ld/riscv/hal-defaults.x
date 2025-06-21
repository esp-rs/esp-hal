ENTRY(_start)

PROVIDE(_stext = ORIGIN(ROTEXT));
PROVIDE(_max_hart_id = 0);

PROVIDE(UserSoft = DefaultHandler);
PROVIDE(SupervisorSoft = DefaultHandler);
PROVIDE(MachineSoft = DefaultHandler);
PROVIDE(UserTimer = DefaultHandler);
PROVIDE(SupervisorTimer = DefaultHandler);
PROVIDE(MachineTimer = DefaultHandler);
PROVIDE(UserExternal = DefaultHandler);
PROVIDE(SupervisorExternal = DefaultHandler);
PROVIDE(MachineExternal = DefaultHandler);

PROVIDE(ExceptionHandler = DefaultExceptionHandler);

PROVIDE(DefaultHandler = EspDefaultHandler);

PROVIDE(interrupt1 = DefaultHandler);
PROVIDE(interrupt2 = DefaultHandler);
PROVIDE(interrupt3 = DefaultHandler);
PROVIDE(interrupt4 = DefaultHandler);
PROVIDE(interrupt5 = DefaultHandler);
PROVIDE(interrupt6 = DefaultHandler);
PROVIDE(interrupt7 = DefaultHandler);
PROVIDE(interrupt8 = DefaultHandler);
PROVIDE(interrupt9 = DefaultHandler);
PROVIDE(interrupt10 = DefaultHandler);
PROVIDE(interrupt11 = DefaultHandler);
PROVIDE(interrupt12 = DefaultHandler);
PROVIDE(interrupt13 = DefaultHandler);
PROVIDE(interrupt14 = DefaultHandler);
PROVIDE(interrupt15 = DefaultHandler);
PROVIDE(interrupt16 = DefaultHandler);
PROVIDE(interrupt17 = DefaultHandler);
PROVIDE(interrupt18 = DefaultHandler);
PROVIDE(interrupt19 = DefaultHandler);
PROVIDE(interrupt20 = DefaultHandler);
PROVIDE(interrupt21 = DefaultHandler);
PROVIDE(interrupt22 = DefaultHandler);
PROVIDE(interrupt23 = DefaultHandler);
PROVIDE(interrupt24 = DefaultHandler);
PROVIDE(interrupt25 = DefaultHandler);
PROVIDE(interrupt26 = DefaultHandler);
PROVIDE(interrupt27 = DefaultHandler);
PROVIDE(interrupt28 = DefaultHandler);
PROVIDE(interrupt29 = DefaultHandler);
PROVIDE(interrupt30 = DefaultHandler);
PROVIDE(interrupt31 = DefaultHandler);

/* not included by esp-hal? */
INCLUDE "device.x"
