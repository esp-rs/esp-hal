pub mod mac;
pub mod phy;
pub mod dma;
pub mod types;

pub use mac::{EthernetMac, MacConfig};
pub use phy::{EthernetPhy, PhyConfig};
