memset = 0x400004b8;
memcpy = 0x400004bc;
memmove = 0x400004c0;
memcmp = 0x400004c4;

strncmp = 0x400004d4;
strncpy = 0x400004cc;
strcpy = 0x400004c8;

abs = 0x40000588;

PROVIDE(strcat = 0x4000053c);
PROVIDE(strcmp = 0x400004d0);
PROVIDE(strchr = 0x40000544);
PROVIDE(strlcpy = 0x40000554);
PROVIDE(strstr = 0x400004dc);
PROVIDE(strcasecmp = 0x40000534);
PROVIDE(memchr = 0x4000052c);

EXTERN(__mktime);
EXTERN(__strnlen);
EXTERN(__atoi);

PROVIDE ( strnlen = __strnlen );
PROVIDE ( atoi = __atoi );
PROVIDE ( mktime = __mktime );

syscall_table_ptr = 0x4085ffd4;
