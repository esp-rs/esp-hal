//! UART UHCI test, blocking

//% CHIPS: esp32c6
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::dma::DmaRxBuf;
use esp_hal::dma::DmaTxBuf;
use esp_hal::dma_buffers;
use esp_hal::uart::uhci::Uhci;
use esp_hal::{
    Blocking,
    uart::{self, Uart},
};

use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

struct Context {
    uhci: Uhci<'static, Blocking>,
    dma_rx: DmaRxBuf,
    dma_tx: DmaTxBuf,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let uart = Uart::new(peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx);

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4092);
        let dma_rx = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut uhci = Uhci::new(uart, peripherals.UHCI0, peripherals.DMA_CH0);
        uhci.apply_config(&uart::uhci::Config::default().with_chunk_limit(dma_rx.len() as u16))
            .unwrap();

        Context {
            uhci,
            dma_rx,
            dma_tx,
        }
    }

    #[test]
    fn test_send_receive(mut ctx: Context) {
        const SEND: &[u8] = b"Hello ESP32";
        ctx.dma_tx.as_mut_slice()[0..SEND.len()].copy_from_slice(&SEND);
        ctx.dma_tx.set_length(SEND.len());

        let (uhci_rx, uhci_tx) = ctx.uhci.split();
        let transfer_rx = uhci_rx
            .read(ctx.dma_rx)
            .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
        let transfer_tx = uhci_tx
            .write(ctx.dma_tx)
            .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
        let (res, _uhci_tx, _dma_tx) = transfer_tx.wait();
        res.unwrap();
        let (res, _uhci_rx, dma_rx) = transfer_rx.wait();
        res.unwrap();

        assert_eq!(
            &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
            SEND
        );
    }

    #[test]
    fn test_long_strings(mut ctx: Context) {
        const LONG_TEST_STRING: &str = "Loremipsumdolorsitamet,consecteturadipiscingelit.Suspendissemetusnisl,pretiumsedeuismodeget,bibendumeusem.Donecaccumsanrisusnibh,etefficiturnisivehiculatempus.Etiamegestasenimatduieleifendmaximus.Nuncinsemperest.Etiamvelodioultrices,interdumeratsed,dignissimmetus.Phasellusexleo,eleifendquisexid,laciniavenenatisneque.Sednuncdiam,molestieveltinciduntnec,ornareetnisi.Maecenasetmolestietortor.Nullaeupulvinarquam.Aeneanmolestieliberoquistortorviverralobortis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.InLoremipsumdolorsitamet,consecteturadipiscingelit.Suspendissemetusnisl,pretiumsedeuismodeget,bibendumeusem.Donecaccumsanrisusnibh,etefficiturnisivehiculatempus.Etiamegestasenimatduieleifendmaximus.Nuncinsemperest.Etiamvelodioultrices,interdumeratsed,dignissimmetus.Phasellusexleo,eleifendquisexid,laciniavenenatisneque.Sednuncdiam,molestieveltinciduntnec,ornareetnisi.Maecenasetmolestietortor.Nullaeupulvinarquam.Aeneanmolestieliberoquistortorviverralobortis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.Inefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunecenim.Proinvenenatistortorveltristiquealiquam.Utelementumtellusligula,velauctorexfermentuma.Vestibulummaximusanteinvulputateornare.Sedquisnislaligulaporttitorfacilisismattissedmi.Crasconsecteturexegetsagittisfeugiat.Invenenatisminectinciduntaliquet.Sedcommodonecorciidvenenatis.Vestibulumanteipsumprimisinfaucibusorciluctusetultricesposuerecubiliacurae;Phasellusinterdumorcefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunecenim.Proinvenenatistortorveltristiquealiquam.Utelementumtellusligula,velauctorexfermentuma.Vestibulummaximusanteinvulputateornare.Sedquisnislaligulaporttitorfacilisismattissedmi.Crasconsecteturexegetsagittisfeugiat.Invenenatisminectinciduntaliquet.Sedcommodonecorciidvenenatis.Vestibulumanteipsumprimisinfaucibusorciluctusetultricesposuerecubiliacurae;Phasellusinterdumorcrtis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.Inefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunece";
        ctx.dma_tx.as_mut_slice()[0..LONG_TEST_STRING.len()]
            .copy_from_slice(&LONG_TEST_STRING.as_bytes());
        ctx.dma_tx.set_length(LONG_TEST_STRING.len());

        let (uhci_rx, uhci_tx) = ctx.uhci.split();
        let transfer_rx = uhci_rx
            .read(ctx.dma_rx)
            .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
        let transfer_tx = uhci_tx
            .write(ctx.dma_tx)
            .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
        let (res, _uhci_tx, _dma_tx) = transfer_tx.wait();
        res.unwrap();
        let (res, _uhci_rx, dma_rx) = transfer_rx.wait();
        res.unwrap();

        assert_eq!(
            &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
            LONG_TEST_STRING.as_bytes()
        );
    }
}
