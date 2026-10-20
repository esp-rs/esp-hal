memset  = 0x2f8004c0;
memcpy  = 0x2f8004c4;
memmove = 0x2f8004c8;
memcmp  = 0x2f8004cc;

strncmp = 0x2f8004dc;
strncpy = 0x2f8004d4;
strcpy  = 0x2f8004d0;

abs = 0x2f800584;

PROVIDE(strcat     = 0x2f800540);
PROVIDE(strcmp     = 0x2f8004d8);
PROVIDE(strchr     = 0x2f800544);
PROVIDE(strlcpy    = 0x2f800554);
PROVIDE(strstr     = 0x2f8004e4);
PROVIDE(strcasecmp = 0x2f800538);
PROVIDE(memchr     = 0x2f800530);

EXTERN(__strnlen);
EXTERN(__atoi);
EXTERN(__mktime);

PROVIDE(strnlen = __strnlen);
PROVIDE(atoi    = __atoi);
PROVIDE(mktime  = __mktime);

syscall_table_ptr = 0x2f07ffc8;
