memset = 0x400011e8;
memcpy = 0x400011f4;
memmove = 0x40001200;
memcmp = 0x4000120c;

strcpy = 0x40001218;
strncpy = 0x40001224;
strncmp = 0x4000123c;

bzero = 0x40001260;

PROVIDE(cache_dbus_mmu_set = 0x400019b0);
PROVIDE( Cache_Suspend_DCache_Autoload = 0x40001734 );
PROVIDE( Cache_Suspend_DCache = 0x400018b4 );
