pub enum EthernetSpeed {
    Speed10M,
    Speed100M,
}

pub enum EthernetDuplex {
    Half,
    Full,
}

pub struct EthernetConfig {
    pub speed: EthernetSpeed,
    pub duplex: EthernetDuplex,
    pub mac_address: [u8; 6],
}

pub struct EthernetPeripheral {
    config: EthernetConfig,
    enabled: bool,
}

impl EthernetPeripheral {
    pub const fn new(config: EthernetConfig) -> Self {
        Self {
            config,
            enabled: false,
        }
    }

    pub fn init(&mut self) -> Result<(), &'static str> {
        if self.enabled {
            return Err("Ethernet peripheral already initialized");
        }
        self.enabled = true;
        Ok(())
    }

    pub fn transmit(&self, packet: &[u8]) -> Result<usize, &'static str> {
        if !self.enabled {
            return Err("Ethernet peripheral not enabled");
        }
        if packet.is_empty() {
            return Err("Cannot transmit empty packet");
        }
        Ok(packet.len())
    }

    pub fn receive<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8], &'static str> {
        if !self.enabled {
            return Err("Ethernet peripheral not enabled");
        }
        if buffer.is_empty() {
            return Err("Receive buffer is empty");
        }
        Ok(&[]
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}
