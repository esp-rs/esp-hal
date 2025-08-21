//! UART UHCI test, async

//% CHIPS: esp32c6
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use esp_hal::{
    Async,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    uart::{self, Uart, uhci::Uhci},
};

esp_bootloader_esp_idf::esp_app_desc!();

struct Context {
    uhci: Uhci<'static, Async>,
    dma_rx: DmaRxBuf,
    dma_tx: DmaTxBuf,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use super::*;

    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let uart = Uart::new(peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx);

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4092);
        let dma_rx = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut uhci = Uhci::new(uart, peripherals.UHCI0, peripherals.DMA_CH0).into_async();
        uhci.apply_config(&uart::uhci::Config::default().with_chunk_limit(dma_rx.len() as u16))
            .unwrap();

        Context {
            uhci,
            dma_rx,
            dma_tx,
        }
    }

    #[test]
    async fn test_send_receive(mut ctx: Context) {
        const SEND: &[u8] = b"Hello ESP32";
        ctx.dma_tx.as_mut_slice()[0..SEND.len()].copy_from_slice(&SEND);
        ctx.dma_tx.set_length(SEND.len());
        let mut transfer = ctx
            .uhci
            .read_write_transfer(ctx.dma_rx, ctx.dma_tx)
            .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
        transfer.wait_for_done().await;
        let (res, _uhci, dma_rx, _dma_tx) = transfer.wait();
        res.unwrap();

        assert_eq!(
            &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
            SEND
        );
    }

    #[test]
    async fn test_long_strings(mut ctx: Context) {
        const LONG_TEST_STRING: &str = "Loremipsumdolorsitamet,consecteturadipiscingelit.Suspendissemetusnisl,pretiumsedeuismodeget,bibendumeusem.Donecaccumsanrisusnibh,etefficiturnisivehiculatempus.Etiamegestasenimatduieleifendmaximus.Nuncinsemperest.Etiamvelodioultrices,interdumeratsed,dignissimmetus.Phasellusexleo,eleifendquisexid,laciniavenenatisneque.Sednuncdiam,molestieveltinciduntnec,ornareetnisi.Maecenasetmolestietortor.Nullaeupulvinarquam.Aeneanmolestieliberoquistortorviverralobortis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.InLoremipsumdolorsitamet,consecteturadipiscingelit.Suspendissemetusnisl,pretiumsedeuismodeget,bibendumeusem.Donecaccumsanrisusnibh,etefficiturnisivehiculatempus.Etiamegestasenimatduieleifendmaximus.Nuncinsemperest.Etiamvelodioultrices,interdumeratsed,dignissimmetus.Phasellusexleo,eleifendquisexid,laciniavenenatisneque.Sednuncdiam,molestieveltinciduntnec,ornareetnisi.Maecenasetmolestietortor.Nullaeupulvinarquam.Aeneanmolestieliberoquistortorviverralobortis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.Inefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunecenim.Proinvenenatistortorveltristiquealiquam.Utelementumtellusligula,velauctorexfermentuma.Vestibulummaximusanteinvulputateornare.Sedquisnislaligulaporttitorfacilisismattissedmi.Crasconsecteturexegetsagittisfeugiat.Invenenatisminectinciduntaliquet.Sedcommodonecorciidvenenatis.Vestibulumanteipsumprimisinfaucibusorciluctusetultricesposuerecubiliacurae;Phasellusinterdumorcefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunecenim.Proinvenenatistortorveltristiquealiquam.Utelementumtellusligula,velauctorexfermentuma.Vestibulummaximusanteinvulputateornare.Sedquisnislaligulaporttitorfacilisismattissedmi.Crasconsecteturexegetsagittisfeugiat.Invenenatisminectinciduntaliquet.Sedcommodonecorciidvenenatis.Vestibulumanteipsumprimisinfaucibusorciluctusetultricesposuerecubiliacurae;Phasellusinterdumorcrtis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.Inefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunece";
        ctx.dma_tx.as_mut_slice()[0..LONG_TEST_STRING.len()]
            .copy_from_slice(&LONG_TEST_STRING.as_bytes());
        ctx.dma_tx.set_length(LONG_TEST_STRING.len());
        let mut transfer = ctx
            .uhci
            .read_write_transfer(ctx.dma_rx, ctx.dma_tx)
            .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
        transfer.wait_for_done().await;
        let (res, _uhci, dma_rx, _dma_tx) = transfer.wait();
        res.unwrap();

        assert_eq!(
            &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
            LONG_TEST_STRING.as_bytes()
        );
    }
}
