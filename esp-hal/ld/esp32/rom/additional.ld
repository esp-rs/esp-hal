memcmp = 0x4000c260;
memcpy = 0x4000c2c8;
memmove = 0x4000c3c0;
memset = 0x4000c44c;

PROVIDE ( strcpy = 0x400013ac );
PROVIDE ( abs = 0x40056340 );
PROVIDE ( strncpy = 0x400015d4 );
PROVIDE ( strncmp = 0x4000c5f4 );

PROVIDE ( bzero = 0x4000c1f4 );

PROVIDE ( strcat = 0x4000c518 );
PROVIDE ( strcmp = 0x40001274 );
PROVIDE ( strchr = 0x4000c53c );
PROVIDE ( strlcpy = 0x4000c584 );
PROVIDE ( strstr = 0x4000c674 );
PROVIDE ( strcasecmp = 0x400011cc );
PROVIDE ( strdup = 0x4000143c );
PROVIDE ( atoi = 0x400566c4 );