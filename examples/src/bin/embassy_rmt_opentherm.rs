//! Demonstrates decoding pulse sequences with RMT
//! Connect GPIO27

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: async embassy embassy-generic-timers

#![no_std]
#![no_main]

use core::cell::RefCell;
use heapless::Vec;

use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Input, Pull, Io, Level, Output, AnyInput},
    peripherals::Peripherals,
    prelude::*,
    Async,
    rmt::{asynch::{RxChannelAsync, TxChannelAsync}, PulseCode, Rmt, TxChannelConfig, RxChannelConfig,
        RxChannelCreatorAsync, TxChannelCreatorAsync},
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_println::{print, println};

use opentherm_boiler_controller_lib::{BoilerControl, TimeBaseRef, Instant};
use opentherm_boiler_controller_lib::opentherm_interface::OpenThermEdgeTriggerBus;
use opentherm_boiler_controller_lib::opentherm_interface::{DataOt, Error as OtError, OpenThermMessage, MessageType};
use opentherm_boiler_controller_lib::opentherm_interface::edge_trigger_capture_interface::{
    CaptureError, EdgeCaptureInterface, EdgeTriggerInterface, InitLevel, TriggerError,
};
use opentherm_boiler_controller_lib::opentherm_interface::manchester;
use opentherm_boiler_controller_lib::opentherm_interface::manchester::{
    manchester_decode, ManchesterDecodingVariant, ManchesterIteratorAdapter,
};
use opentherm_boiler_controller_lib::opentherm_interface::{
    CAPTURE_OT_FRAME_PAYLOAD_SIZE, MANCHESTER_OPENTHERM_RESOLUTION, MESSAGE_DATA_ID_BIT_LEN,
    MESSAGE_DATA_VALUE_BIT_LEN, MESSAGE_TYPE_BIT_LEN, OT_FRAME_SKIP_SPARE,
};
use opentherm_boiler_controller_lib::opentherm_interface::api;
use opentherm_boiler_controller_lib::opentherm_interface::api::OpenThermBus;
use opentherm_boiler_controller_lib::opentherm_interface::OpenThermInterface;

pub struct EspOpenthermRmt<E: EdgeCaptureInterface, T: EdgeTriggerInterface>{
    edge_capture_drv: E,
    edge_trigger_drv: T,
    send_count: usize,
}

const TOTAL_CAPTURE_OT_FRAME_SIZE: usize = 34usize;

impl<E: EdgeCaptureInterface,T: EdgeTriggerInterface> EspOpenthermRmt<E, T> {
    pub fn new( edge_capture_driver: E, edge_trigger_driver: T) -> Self {
        Self{
            edge_capture_drv: edge_capture_driver,
            edge_trigger_drv: edge_trigger_driver,
            send_count: 0,
        }
    }
}

impl<E: EdgeCaptureInterface,T: EdgeTriggerInterface> EdgeTriggerInterface for EspOpenthermRmt<E, T> {
    async fn trigger(
        &mut self,
        iterator: impl Iterator<Item = bool>,
        period: core::time::Duration,
    ) -> Result<(), TriggerError> {
        self.edge_trigger_drv
            .trigger(iterator, period)
            .await
    }
}

const VEC_SIZE_OT: usize = 128usize;
// Inverted - invert polarity of the signal
// N - output vec size
impl<E: EdgeCaptureInterface,T: EdgeTriggerInterface> EdgeCaptureInterface<VEC_SIZE_OT> for EspOpenthermRmt<E, T> {
//  impl<const N: usize, const Inverted: bool> EdgeCaptureInterface<N>
    //  for RmtEdgeCapture<N, Inverted>
    //
    //  TODO:
    //  make a list
    async fn start_capture(
        &mut self,
        timeout_inactive_capture: core::time::Duration,
        timeout_till_active_capture: core::time::Duration,
    ) -> Result<(InitLevel, Vec<core::time::Duration, VEC_SIZE_OT>), CaptureError> {

        self.edge_capture_drv.start_capture(timeout_inactive_capture, timeout_till_active_capture).await
    }
}

//  Used to pull in the implementation of OpenThermInterface:
impl<E: EdgeCaptureInterface,T: EdgeTriggerInterface> OpenThermEdgeTriggerBus for EspOpenthermRmt<E, T> {}

//  This part is use to connect pure trait implementation with our specific class which makes this
//  ugly wrapper:
impl<E: EdgeCaptureInterface,T: EdgeTriggerInterface> OpenThermInterface for EspOpenthermRmt<E, T> {
    async fn send(&mut self, message: OpenThermMessage) -> Result<(), OtError> {
        OpenThermEdgeTriggerBus::send(self, message).await
    }
    async fn listen(&mut self) -> Result<OpenThermMessage, OtError> {
        OpenThermEdgeTriggerBus::listen(self).await
    }
    async fn write(&mut self, cmd: DataOt) -> Result<(), OtError> {
        todo!()
    }
    async fn read(&mut self, cmd: DataOt) -> Result<DataOt, OtError> {
        todo!()
    }
}

pub struct RmtEdgeTrigger {
    rmt_channel_tx: esp_hal::rmt::Channel<Async,1>,
}

impl RmtEdgeTrigger {
    pub fn new(mut rmt_channel: esp_hal::rmt::Channel<Async,1>) -> Self {
        println!("Create new RmtEdgeTrigger dev");
        Self { rmt_channel_tx: rmt_channel }
    }
}

pub struct RmtEdgeCapture<const N: usize, const Inverted: bool = false> {
    //  input_pin: Input<'d>,
    rmt_channel_rx: esp_hal::rmt::Channel<Async, 0>,
}

impl<'d, const N: usize, const Inverted: bool> RmtEdgeCapture<N, Inverted> {
    pub fn new(mut rmt_channel: esp_hal::rmt::Channel<Async,0>) -> Self {
        Self { rmt_channel_rx: rmt_channel }
    }
    #[inline]
    fn insert(
        vector: &mut Vec<core::time::Duration, N>,
        period: core::time::Duration,
    ) -> Result<(), core::time::Duration> {
        vector.insert(0, period)
        //  vector.push(period)
    }
}


//  _____|''|_____|''|__|''|__    ___|''|__|'''''|__|''|___   //  0x200000003
//  -----|  1  |  0  |  0  |      |  0  |  0  |  1  |  1  |
impl EdgeTriggerInterface for RmtEdgeTrigger {
    async fn trigger(
        &mut self,
        iterator: impl Iterator<Item = bool>,
        period: core::time::Duration,
    ) -> Result<(), TriggerError> {

        println!("call trigger for RmtEdgeTrigger dev");
        let period = period.as_nanos() as u16;
        let period = 625u16;
        let mut data = [PulseCode::default(); 48];
        //  {
        //      level1: true,
        //      length1: 100,
        //      level2: false,
        //      length2: 20,
        //  }
        //  data[data.len() - 1] = PulseCode::default();

        for (i, entry) in iterator.enumerate() {
            match i%2 {
                0 => {
                    data[i/2].level1 = true;
                    data[i/2].length1 = period;
                },
                _ => {
                    data[i/2].level2 = false;
                    data[i/2].length2 = period;
                }
            }
        }

        for (i, entry) in data[..data.len()].iter().enumerate() {
            println!("[{i}],e={},l={}, e2={},l2={}", entry.level1, entry.length1, entry.level2, entry.length2);
        }

        println!("transmit");
        self.rmt_channel_tx.transmit(&data).await.unwrap();

        //  let period = MANCHESTER_OPENTHERM_RESOLUTION;
        //  self.output_pin.set_low(); //  generate idle state:
        //  Timer::after(3 * convert_duration_to_embassy(period)).await; //  await one period in idle state

        //  log::info!("Edge Trigger sent count: {count}");

        //  self.output_pin.set_low(); //  generate idle state after
        //  Timer::after(convert_duration_to_embassy(period)).await; //  await one period in idle state
        Ok(())
    }
}

// Inverted - invert polarity of the signal
// N - output vec size
impl<const N: usize, const Inverted: bool> EdgeCaptureInterface<N>
    for RmtEdgeCapture<N, Inverted>
{
    //  TODO:
    //  make a list
    async fn start_capture(
        &mut self,
        timeout_inactive_capture: core::time::Duration,
        timeout_till_active_capture: core::time::Duration,
    ) -> Result<(InitLevel, Vec<core::time::Duration, N>), CaptureError> {
        // let init_state = match self.input_pin.is_high() {
        //     true => InitLevel::High,
        //     false => InitLevel::Low,
        // };

        //  let mut capture_timestamp = start_timestamp;
        //  let mut current_level = init_state.clone();
        let mut timestamps = Vec::<core::time::Duration, N>::new();

        todo!()

        //  log::info!("Return InitLevel: {:?}", init_state);
        //  Ok((init_state, timestamps))
    }
}

const WIDTH: usize = 80;
const RMT_CLK_DIV: u8 = 64;

static RECEIVED_COUNT: Mutex<RefCell<Option<u32>>> =
    Mutex::new(RefCell::new(None));

static RECEIVED_DATA : critical_section::Mutex<RefCell<Option<[PulseCode; 48]>>> =
    Mutex::new(RefCell::new(
        Some([PulseCode {
                level1: true,
                length1: 0,
                level2: false,
                length2: 0,
            }; 48]) ));

#[cfg(debug_assertions)]
compile_error!("Run this example in release mode");

#[embassy_executor::task]
async fn signal_task(mut channel: esp_hal::rmt::Channel<Async, 1>, button: AnyInput<'static>) {
    let mut level = false;
    loop {
        let new_level = button.is_high();
        if level != new_level {
            level = new_level;
            if new_level == true {
                Timer::after(Duration::from_millis(500)).await;
                continue;
            }
            let (count, data) = critical_section::with(|cs| {
                let count = RECEIVED_COUNT.borrow_ref(cs);
                if *count == None {
                    return (0, None);
                }
                let data = RECEIVED_DATA.borrow_ref(cs);
                let ret_data = match *data {
                    Some(data) => {
                        println!("borrow_ref Some(data)");
                        let mut ret_data = [PulseCode::default(); 48];

                        for (i, entry) in data[..data.len()].iter().enumerate() {
                            ret_data[i].level1 = !(*entry).level1;
                            ret_data[i].length1 = (*entry).length1;
                            ret_data[i].level2 = !(*entry).level2;
                            ret_data[i].length2 = (*entry).length2;
                        }
                        ret_data[((*count).unwrap_or(0u32)) as usize] = PulseCode::default();
                        Some(ret_data)
                    },
                    None => {None}
                };

                let count = (*count).unwrap_or(0);
                (count, ret_data)
            });

            println!("button {new_level}, count: {count}");
            if let Some(data) = data {
                //  Send it to the ether:
                channel.transmit(&data).await.unwrap();
                let mut total = 0usize;
                let mut i = 0;
                for entry in &data[..data.len()] {
                    println!("[{i}]: e.l1 = {}, e.l2 ={}", entry.length1, entry.length2);
                    i+=1;
                    if entry.length1 == 0 {
                        break;
                    }
                    total += entry.length1 as usize;

                    if entry.length2 == 0 {
                        break;
                    }
                    total += entry.length2 as usize;
                }
                println!("data size {}, total: {}", data.len(), total);
                for entry in &data[..data.len()] {
                    if entry.length1 == 0 {
                        break;
                    }

                    let count = WIDTH / (total / entry.length1 as usize);
                    let c = if entry.level1 { '-' } else { '_' };
                    for _ in 0..count + 1 {
                        print!("{}", c);
                    }

                    if entry.length2 == 0 {
                        break;
                    }

                    let count = WIDTH / (total / entry.length2 as usize);
                    let c = if entry.level2 { '-' } else { '_' };
                    for _ in 0..count + 1 {
                        print!("{}", c);
                    }
                }
                println!();

            }
            else
            {
                println!("Missing data");
            }
            Timer::after(Duration::from_millis(1000)).await;
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

struct EspTime {}

impl EspTime {
    pub fn new() -> Self {
        Self{}
    }
}
impl TimeBaseRef for EspTime {
    fn now(&self) -> Instant {
        todo!()
    }
}

#[main]
async fn main(spawner: Spawner) {
    println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timer_group0.timer0);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32h2")] {
            let freq = 32.MHz();
        } else {
            let freq = 80.MHz();
        }
    };

    {
        //  let mut input = Input::new(io.pins.gpio27, Pull::Down);
        //  let mut input = Input::new(io.pins.gpio4, Pull::Down);
    }

    let mut button = AnyInput::new(io.pins.gpio0, Pull::Up);
    let mut led = Output::new(io.pins.gpio2, Level::High);

    let rmt = Rmt::new_async(peripherals.RMT, freq, &clocks).unwrap();
    let rx_config = RxChannelConfig {
        clk_divider: RMT_CLK_DIV,
        idle_threshold: 10000,
        ..RxChannelConfig::default()
    };

    let mut channel_tx =
        TxChannelCreatorAsync::configure(rmt.channel1, io.pins.gpio27,
            TxChannelConfig{
                clk_divider: RMT_CLK_DIV,
                idle_output_level: false,
                idle_output: false,
                carrier_modulation: false,
                //  carrier_high: 0u16,
                //  carrier_low: 0u16,
                carrier_high: 1050u16,
                carrier_low: 1050u16,
                carrier_level: true,
        }).unwrap();

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
            let mut channel_rx = RxChannelCreatorAsync::configure(rmt.channel0, io.pins.gpio4, rx_config).unwrap();
        } else if #[cfg(feature = "esp32s3")] {
            let mut channel_rx = rmt.channel7.configure(io.pins.gpio4, rx_config).unwrap();
        } else {
            let mut channel_rx = rmt.channel2.configure(io.pins.gpio4, rx_config).unwrap();
        }
    }

    let mut rmt_tx = RmtEdgeTrigger::new(channel_tx);
    let mut rmt_rx: RmtEdgeCapture<128> = RmtEdgeCapture::new(channel_rx);

    let opentherm_device = EspOpenthermRmt::new(rmt_rx, rmt_tx);
    let esp_time = EspTime::new();

    let mut boiler = BoilerControl::new(opentherm_device, esp_time);

    Timer::after(Duration::from_millis(500)).await;

    for i in 1..7 {
        led.toggle();
        Timer::after(Duration::from_millis(100)).await;
    }

    println!("Start loop");
    loop {
        println!("send the trigger data");
        //  rmt_tx.trigger(data.clone().into_iter(), core::time::Duration::from_millis(500)).await;
        //  input.wait_for_falling_edge().await;
        println!("loop");
        Timer::after(Duration::from_millis(1000)).await;
    }
}
