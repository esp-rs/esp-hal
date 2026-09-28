memset = 0x40000498;
memcpy = 0x4000049c;
memmove = 0x400004a0;
memcmp = 0x400004a4;

strncmp = 0x400004b4;
strncpy = 0x400004ac;
strcpy = 0x400004a8;

abs = 0x4000055c;

PROVIDE( strcat = 0x40000518 );
PROVIDE( strcmp = 0x400004b0 );
PROVIDE( strchr = 0x4000051c );
PROVIDE( strlcpy = 0x4000052c );
PROVIDE( strstr = 0x400004bc );
PROVIDE( strcasecmp = 0x40000510 );

PROVIDE( memchr = 0x40000508 );

EXTERN(__mktime);
EXTERN(__strnlen);
EXTERN(__atoi);

PROVIDE ( strnlen = __strnlen );
PROVIDE ( atoi = __atoi );
PROVIDE ( mktime = __mktime );

syscall_table_ptr = 0x4085ffd0;
