//! UART UHCI test, async

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s3
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

#[cfg(riscv)]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    peripherals::Peripherals,
    timer::timg::TimerGroup,
    uart::{self, Uart, uhci::Uhci},
};

struct Context {
    dma_rx: DmaRxBuf,
    dma_tx: DmaTxBuf,
    peripherals: Peripherals,
}

const LONG_TEST_STRING: &str = "Loremipsumdolorsitamet,consecteturadipiscingelit.Suspendissemetusnisl,pretiumsedeuismodeget,bibendumeusem.Donecaccumsanrisusnibh,etefficiturnisivehiculatempus.Etiamegestasenimatduieleifendmaximus.Nuncinsemperest.Etiamvelodioultrices,interdumeratsed,dignissimmetus.Phasellusexleo,eleifendquisexid,laciniavenenatisneque.Sednuncdiam,molestieveltinciduntnec,ornareetnisi.Maecenasetmolestietortor.Nullaeupulvinarquam.Aeneanmolestieliberoquistortorviverralobortis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.InLoremipsumdolorsitamet,consecteturadipiscingelit.Suspendissemetusnisl,pretiumsedeuismodeget,bibendumeusem.Donecaccumsanrisusnibh,etefficiturnisivehiculatempus.Etiamegestasenimatduieleifendmaximus.Nuncinsemperest.Etiamvelodioultrices,interdumeratsed,dignissimmetus.Phasellusexleo,eleifendquisexid,laciniavenenatisneque.Sednuncdiam,molestieveltinciduntnec,ornareetnisi.Maecenasetmolestietortor.Nullaeupulvinarquam.Aeneanmolestieliberoquistortorviverralobortis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.Inefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunecenim.Proinvenenatistortorveltristiquealiquam.Utelementumtellusligula,velauctorexfermentuma.Vestibulummaximusanteinvulputateornare.Sedquisnislaligulaporttitorfacilisismattissedmi.Crasconsecteturexegetsagittisfeugiat.Invenenatisminectinciduntaliquet.Sedcommodonecorciidvenenatis.Vestibulumanteipsumprimisinfaucibusorciluctusetultricesposuerecubiliacurae;Phasellusinterdumorcefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunecenim.Proinvenenatistortorveltristiquealiquam.Utelementumtellusligula,velauctorexfermentuma.Vestibulummaximusanteinvulputateornare.Sedquisnislaligulaporttitorfacilisismattissedmi.Crasconsecteturexegetsagittisfeugiat.Invenenatisminectinciduntaliquet.Sedcommodonecorciidvenenatis.Vestibulumanteipsumprimisinfaucibusorciluctusetultricesposuerecubiliacurae;Phasellusinterdumorcrtis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.Inefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunece";
// const MEDIUM_TEST_STRING: &str =
// "Loremipsumdolorsitamet,consecteturadipiscingelit.Suspendissemetusnisl,pretiumsedeuismodeget,
// bibendumeusem.Donecaccumsanrisusnibh";
const SHORT_TEST_STRING: &str = "Hello ESP32";
const DMA_BUFFER_SIZE: u16 = 4092;
const LOOP_COUNT: u32 = 5;
const BAUDRATE: u32 = 921600;
// const BAUDRATES: &[u32] = &[9600, 19200, 28800, 38400, 57600, 76800, 115200, 230400, 460800,
// 576000, 921600];

#[embedded_test::tests(default_timeout = 5, executor = hil_test::Executor::new())]
mod tests {
    use super::*;

    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            dma_buffers!(DMA_BUFFER_SIZE as usize);
        let dma_rx = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        Context {
            dma_rx,
            dma_tx,
            peripherals,
        }
    }

    #[test]
    fn test_send_receive(ctx: Context) {
        #[cfg(riscv)]
        let sw_int = SoftwareInterruptControl::new(ctx.peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_int.software_interrupt0,
        );

        let (rx, tx) = hil_test::common_test_pins!(ctx.peripherals);
        let uart = Uart::new(ctx.peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx);
        let mut uhci = Uhci::new(uart, ctx.peripherals.UHCI0, ctx.peripherals.DMA_CH0);
        uhci.apply_rx_config(&uart::uhci::RxConfig::default().with_chunk_limit(DMA_BUFFER_SIZE))
            .unwrap();
        uhci.apply_tx_config(&uart::uhci::TxConfig::default())
            .unwrap();
        uhci.set_uart_config(&uart::Config::default().with_baudrate(BAUDRATE))
            .unwrap();

        let (uhci_rx, uhci_tx) = uhci.split();
        let mut uhci_rx_opt = Some(uhci_rx);
        let mut uhci_tx_opt = Some(uhci_tx);
        let mut dma_rx_opt = Some(ctx.dma_rx);
        let mut dma_tx_opt = Some(ctx.dma_tx);

        for _ in 0..LOOP_COUNT {
            let uhci_rx = uhci_rx_opt.take().unwrap();
            let uhci_tx = uhci_tx_opt.take().unwrap();
            let dma_rx = dma_rx_opt.take().unwrap();
            let mut dma_tx = dma_tx_opt.take().unwrap();

            dma_tx.as_mut_slice()[0..SHORT_TEST_STRING.len()]
                .copy_from_slice(&SHORT_TEST_STRING.as_bytes());
            dma_tx.set_length(SHORT_TEST_STRING.len());

            let transfer_rx = uhci_rx
                .read(dma_rx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let transfer_tx = uhci_tx
                .write(dma_tx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let (res, uhci_tx, dma_tx) = transfer_tx.wait();
            res.unwrap();
            let (res, uhci_rx, dma_rx) = transfer_rx.wait();
            res.unwrap();

            assert_eq!(
                &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
                SHORT_TEST_STRING.as_bytes()
            );

            uhci_rx_opt = Some(uhci_rx);
            uhci_tx_opt = Some(uhci_tx);
            dma_rx_opt = Some(dma_rx);
            dma_tx_opt = Some(dma_tx);
        }
    }

    #[test]
    fn test_long_strings(ctx: Context) {
        #[cfg(riscv)]
        let sw_int = SoftwareInterruptControl::new(ctx.peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_int.software_interrupt0,
        );

        let (rx, tx) = hil_test::common_test_pins!(ctx.peripherals);
        let uart = Uart::new(ctx.peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx);
        let mut uhci = Uhci::new(uart, ctx.peripherals.UHCI0, ctx.peripherals.DMA_CH0);
        uhci.apply_rx_config(&uart::uhci::RxConfig::default().with_chunk_limit(DMA_BUFFER_SIZE))
            .unwrap();
        uhci.apply_tx_config(&uart::uhci::TxConfig::default())
            .unwrap();
        uhci.set_uart_config(&uart::Config::default().with_baudrate(BAUDRATE))
            .unwrap();

        let (uhci_rx, uhci_tx) = uhci.split();
        let mut uhci_rx_opt = Some(uhci_rx);
        let mut uhci_tx_opt = Some(uhci_tx);
        let mut dma_rx_opt = Some(ctx.dma_rx);
        let mut dma_tx_opt = Some(ctx.dma_tx);

        for _ in 0..LOOP_COUNT {
            let uhci_rx = uhci_rx_opt.take().unwrap();
            let uhci_tx = uhci_tx_opt.take().unwrap();
            let dma_rx = dma_rx_opt.take().unwrap();
            let mut dma_tx = dma_tx_opt.take().unwrap();

            dma_tx.as_mut_slice()[0..LONG_TEST_STRING.len()]
                .copy_from_slice(&LONG_TEST_STRING.as_bytes());
            dma_tx.set_length(LONG_TEST_STRING.len());

            let transfer_rx = uhci_rx
                .read(dma_rx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let transfer_tx = uhci_tx
                .write(dma_tx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let (res, uhci_tx, dma_tx) = transfer_tx.wait();
            res.unwrap();
            let (res, uhci_rx, dma_rx) = transfer_rx.wait();
            res.unwrap();

            assert_eq!(
                &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
                LONG_TEST_STRING.as_bytes()
            );

            uhci_rx_opt = Some(uhci_rx);
            uhci_tx_opt = Some(uhci_tx);
            dma_rx_opt = Some(dma_rx);
            dma_tx_opt = Some(dma_tx);
        }
    }

    #[test]
    async fn test_send_receive_async(ctx: Context) {
        #[cfg(riscv)]
        let sw_int = SoftwareInterruptControl::new(ctx.peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_int.software_interrupt0,
        );

        let (rx, tx) = hil_test::common_test_pins!(ctx.peripherals);
        let uart = Uart::new(ctx.peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx);
        let mut uhci = Uhci::new(uart, ctx.peripherals.UHCI0, ctx.peripherals.DMA_CH0);
        uhci.apply_rx_config(&uart::uhci::RxConfig::default().with_chunk_limit(DMA_BUFFER_SIZE))
            .unwrap();
        uhci.apply_tx_config(&uart::uhci::TxConfig::default())
            .unwrap();
        uhci.set_uart_config(&uart::Config::default().with_baudrate(BAUDRATE))
            .unwrap();

        let uhci = uhci.into_async();
        let (uhci_rx, uhci_tx) = uhci.split();
        let mut uhci_rx_opt = Some(uhci_rx);
        let mut uhci_tx_opt = Some(uhci_tx);
        let mut dma_rx_opt = Some(ctx.dma_rx);
        let mut dma_tx_opt = Some(ctx.dma_tx);

        for _ in 0..LOOP_COUNT {
            let uhci_rx = uhci_rx_opt.take().unwrap();
            let uhci_tx = uhci_tx_opt.take().unwrap();
            let dma_rx = dma_rx_opt.take().unwrap();
            let mut dma_tx = dma_tx_opt.take().unwrap();

            dma_tx.as_mut_slice()[0..SHORT_TEST_STRING.len()]
                .copy_from_slice(&SHORT_TEST_STRING.as_bytes());
            dma_tx.set_length(SHORT_TEST_STRING.len());

            let mut transfer_rx = uhci_rx
                .read(dma_rx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let mut transfer_tx = uhci_tx
                .write(dma_tx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            transfer_tx.wait_for_done().await;
            let (res, uhci_tx, dma_tx) = transfer_tx.wait();
            res.unwrap();
            transfer_rx.wait_for_done().await;
            let (res, uhci_rx, dma_rx) = transfer_rx.wait();
            res.unwrap();

            assert_eq!(
                &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
                SHORT_TEST_STRING.as_bytes()
            );
            uhci_rx_opt = Some(uhci_rx);
            uhci_tx_opt = Some(uhci_tx);
            dma_rx_opt = Some(dma_rx);
            dma_tx_opt = Some(dma_tx);
        }
    }

    #[test]
    async fn test_long_strings_async(ctx: Context) {
        #[cfg(riscv)]
        let sw_int = SoftwareInterruptControl::new(ctx.peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);
        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            sw_int.software_interrupt0,
        );

        let (rx, tx) = hil_test::common_test_pins!(ctx.peripherals);
        let uart = Uart::new(ctx.peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx);
        let mut uhci = Uhci::new(uart, ctx.peripherals.UHCI0, ctx.peripherals.DMA_CH0);
        uhci.apply_rx_config(&uart::uhci::RxConfig::default().with_chunk_limit(DMA_BUFFER_SIZE))
            .unwrap();
        uhci.apply_tx_config(&uart::uhci::TxConfig::default())
            .unwrap();
        uhci.set_uart_config(&uart::Config::default().with_baudrate(BAUDRATE))
            .unwrap();

        let uhci = uhci.into_async();
        let (uhci_rx, uhci_tx) = uhci.split();
        let mut uhci_rx_opt = Some(uhci_rx);
        let mut uhci_tx_opt = Some(uhci_tx);
        let mut dma_rx_opt = Some(ctx.dma_rx);
        let mut dma_tx_opt = Some(ctx.dma_tx);

        for _ in 0..LOOP_COUNT {
            let uhci_rx = uhci_rx_opt.take().unwrap();
            let uhci_tx = uhci_tx_opt.take().unwrap();
            let dma_rx = dma_rx_opt.take().unwrap();
            let mut dma_tx = dma_tx_opt.take().unwrap();

            dma_tx.as_mut_slice()[0..LONG_TEST_STRING.len()]
                .copy_from_slice(&LONG_TEST_STRING.as_bytes());
            dma_tx.set_length(LONG_TEST_STRING.len());

            let mut transfer_rx = uhci_rx
                .read(dma_rx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let mut transfer_tx = uhci_tx
                .write(dma_tx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            transfer_tx.wait_for_done().await;
            let (res, uhci_tx, dma_tx) = transfer_tx.wait();
            res.unwrap();
            transfer_rx.wait_for_done().await;
            let (res, uhci_rx, dma_rx) = transfer_rx.wait();
            res.unwrap();

            assert_eq!(
                &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
                LONG_TEST_STRING.as_bytes()
            );
            uhci_rx_opt = Some(uhci_rx);
            uhci_tx_opt = Some(uhci_tx);
            dma_rx_opt = Some(dma_rx);
            dma_tx_opt = Some(dma_tx);
        }
    }

    // Will be re-enabled and fully implemented once changing uart baudrate after split is supported
    // #[test]
    // async fn test_baudrate_change_rx(ctx: Context) {
    // let (rx1, tx1) = hil_test::common_test_pins!(ctx.peripherals);
    // let (rx2, tx2) = hil_test::i2c_pins!(ctx.peripherals);
    //
    // let uart0 = Uart::new(ctx.peripherals.UART0, uart::Config::default())
    // .unwrap()
    // .with_tx(tx2)
    // .with_rx(rx1);
    // let mut uhci = Uhci::new(uart0, ctx.peripherals.UHCI0, ctx.peripherals.DMA_CH0);
    // uhci.apply_rx_config(&uart::uhci::RxConfig::default().with_chunk_limit(DMA_BUFFER_SIZE))
    // .unwrap();
    // uhci.apply_tx_config(&uart::uhci::TxConfig::default())
    // .unwrap();
    //
    // let mut uart1 = Uart::new(ctx.peripherals.UART1, uart::Config::default())
    // .unwrap()
    // .with_tx(tx1)
    // .with_rx(rx2);
    //
    // let uhci = uhci.into_async();
    //
    // let (uhci_rx, uhci_tx) = uhci.split();
    // let mut uhci_rx_opt = Some(uhci_rx);
    // let mut uhci_tx_opt = Some(uhci_tx);
    // let mut dma_rx_opt = Some(ctx.dma_rx);
    // for baudrate in BAUDRATES {
    // let mut uhci_rx = uhci_rx_opt.take().unwrap();
    // let mut uhci_tx = uhci_tx_opt.take().unwrap();
    // let dma_rx = dma_rx_opt.take().unwrap();
    //
    // let config = &uart::Config::default().with_baudrate(*baudrate);
    // uart1.apply_config(config).unwrap();
    // THE CONFIG IS A LIE
    // :(
    // uhci_rx.uart_rx.apply_config(config).unwrap();
    // uhci_tx.uart_tx.apply_config(config).unwrap();
    // let mut transfer_rx = uhci_rx
    // .read(dma_rx)
    // .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
    // uart1.write(MEDIUM_TEST_STRING.as_bytes()).unwrap();
    // transfer_rx.wait_for_done().await;
    // let (res, uhci_rx, dma_rx) = transfer_rx.wait();
    // res.unwrap();
    // assert_eq!(
    // &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
    // MEDIUM_TEST_STRING.as_bytes()
    // );
    // uhci_rx_opt = Some(uhci_rx);
    // uhci_tx_opt = Some(uhci_tx);
    // dma_rx_opt = Some(dma_rx);
    // }
    // }
    //
    // #[test]
    // async fn test_baudrate_change_tx(ctx: Context) {
    // return;
    // }
}
