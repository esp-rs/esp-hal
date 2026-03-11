memset = 0x40000354;
memcpy = 0x40000358;
memmove = 0x4000035c;
memcmp = 0x40000360;

strncmp = 0x40000370;
strncpy = 0x40000368;
strcpy = 0x40000364;

abs = 0x40000424;

PROVIDE(cache_dbus_mmu_set = 0x40000564);

PROVIDE( strcat = 0x400003d8 );
PROVIDE( strcmp = 0x4000036c );
PROVIDE( strchr = 0x400003e0 );
PROVIDE( strlcpy = 0x400003f0 );
PROVIDE( strstr = 0x40000378 );
PROVIDE( strcasecmp = 0x400003d0 );

PROVIDE( memchr = 0x400003c8 );

EXTERN(__mktime);
EXTERN(__strnlen);
EXTERN(__atoi);

PROVIDE ( strnlen = __strnlen );
PROVIDE ( atoi = __atoi );
PROVIDE ( mktime = __mktime );

syscall_table_ptr = 0x3fcdffe0;
