memset = 0x400004a0;
memcpy = 0x400004a4;
memmove = 0x400004a8;
memcmp = 0x400004ac;

strncmp = 0x400004bc;
strncpy = 0x400004b4;
strcpy = 0x400004b0;

abs = 0x40000570;

PROVIDE( strcat = 0x40000524 );
PROVIDE( strcmp = 0x400004b8 );
PROVIDE( strchr = 0x4000052c );
PROVIDE( strlcpy = 0x4000053c );
PROVIDE( strstr = 0x400004c4 );
PROVIDE( strcasecmp = 0x4000051c );

PROVIDE( memchr = 0x40000514 );
