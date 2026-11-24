memset = 0x40000498;
memcpy = 0x4000049c;
memmove = 0x400004a0;
memcmp = 0x400004a4;

strncmp = 0x400004b4;
strncpy = 0x400004ac;
strcpy = 0x400004a8;

abs = 0x40000568;

PROVIDE( strcat = 0x4000051c );
PROVIDE( strcmp = 0x400004b0 );
PROVIDE( strchr = 0x40000524 );
PROVIDE( strlcpy = 0x40000534 );
PROVIDE( strstr = 0x400004bc );
PROVIDE( strcasecmp = 0x40000514 );

PROVIDE( memchr = 0x4000050c );

EXTERN(__mktime);
EXTERN(__strnlen);
EXTERN(__atoi);

PROVIDE ( strnlen = __strnlen );
PROVIDE ( atoi = __atoi );
PROVIDE ( mktime = __mktime );

syscall_table_ptr = 0x4084ffd4;
