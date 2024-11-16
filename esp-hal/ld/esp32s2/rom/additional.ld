memcmp = 0x4001ab40;
memcpy = 0x4001aba8;
memmove = 0x4001acb0;
memset = 0x4001ad3c;

strcpy = 0x40007cfc;
strncpy = 0x40007f20;
strncmp = 0x4001ae64;

bzero = 0x400078c8;

PROVIDE ( cache_dbus_mmu_set = 0x40018eb0 );

PROVIDE ( strcat = 0x4001ad90 );
PROVIDE ( strcmp = 0x40007be4 );
PROVIDE ( strchr = 0x4001adb0 );
PROVIDE ( strlcpy = 0x4001adf8 );
PROVIDE ( strstr = 0x4001aee8 );
PROVIDE ( strcasecmp = 0x40007b38 );
PROVIDE ( strdup = 0x40007d84 );
