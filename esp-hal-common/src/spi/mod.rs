pub mod master;
#[cfg(all(any(spi0, spi1, spi2, spi3), not(pdma)))]
pub mod slave;
