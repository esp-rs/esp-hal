memset = 0x40000488;
memcpy = 0x4000048c;
memmove = 0x40000490;
memcmp = 0x40000494;

strcpy = 0x40000498;
strncpy = 0x4000049c;
strncmp = 0x400004a4;

PROVIDE ( strcat = 0x4000050c );
PROVIDE ( strcmp = 0x400004a0 );
PROVIDE ( strchr = 0x40000514 );
PROVIDE ( strlcpy = 0x40000524 );
PROVIDE ( strstr = 0x400004ac );
PROVIDE ( strcasecmp = 0x40000504 );
