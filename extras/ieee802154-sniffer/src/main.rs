use std::{
    io::{stdout, BufRead, BufReader, Write},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use clap::Parser;
use lazy_static::lazy_static;
use pcap_file::{
    pcap::{PcapHeader, PcapPacket, PcapWriter},
    DataLink,
};
use r_extcap::{
    config::{ConfigOptionValue, SelectorConfig, StringConfig},
    controls::{synchronous::ExtcapControlSender, ControlCommand, ControlPacket},
    interface::{Dlt, Interface, Metadata},
    ExtcapStep,
};

#[derive(Debug, Parser)]
pub struct AppArgs {
    #[command(flatten)]
    extcap: r_extcap::ExtcapArgs,

    #[arg(long, default_value = "")]
    serialport: String,

    #[arg(long, default_value = "11")]
    channel: String,
}

lazy_static! {
    static ref METADATA: Metadata = Metadata {
        help_url: "http://github.com/esp-rs".into(),
        display_description: "esp-ieee802154".into(),
        ..r_extcap::cargo_metadata!()
    };
    static ref WIFI_CAPTURE_INTERFACE: Interface = Interface {
        value: "802.15.4".into(),
        display: "esp-ieee802154 Sniffer".into(),
        dlt: Dlt {
            data_link_type: DataLink::USER0,
            name: "USER0".into(),
            display: "IEEE802.15.4".into(),
        },
    };
    static ref CONFIG_SERIALPORT: StringConfig = StringConfig::builder()
        .config_number(1)
        .call("serialport")
        .display("Serialport")
        .tooltip("Serialport to connect to")
        .required(false)
        .placeholder("")
        .build();
    static ref CONFIG_CHANNEL: SelectorConfig = SelectorConfig::builder()
        .config_number(3)
        .call("channel")
        .display("Channel")
        .tooltip("Channel Selector")
        .default_options([
            ConfigOptionValue::builder()
                .value("11")
                .display("11")
                .default(true)
                .build(),
            ConfigOptionValue::builder()
                .value("12")
                .display("12")
                .build(),
            ConfigOptionValue::builder()
                .value("13")
                .display("13")
                .build(),
            ConfigOptionValue::builder()
                .value("14")
                .display("14")
                .build(),
            ConfigOptionValue::builder()
                .value("15")
                .display("15")
                .build(),
            ConfigOptionValue::builder()
                .value("16")
                .display("16")
                .build(),
            ConfigOptionValue::builder()
                .value("17")
                .display("17")
                .build(),
            ConfigOptionValue::builder()
                .value("18")
                .display("18")
                .build(),
            ConfigOptionValue::builder()
                .value("19")
                .display("19")
                .build(),
            ConfigOptionValue::builder()
                .value("20")
                .display("20")
                .build(),
            ConfigOptionValue::builder()
                .value("21")
                .display("21")
                .build(),
            ConfigOptionValue::builder()
                .value("22")
                .display("22")
                .build(),
            ConfigOptionValue::builder()
                .value("23")
                .display("23")
                .build(),
            ConfigOptionValue::builder()
                .value("24")
                .display("24")
                .build(),
            ConfigOptionValue::builder()
                .value("25")
                .display("25")
                .build(),
            ConfigOptionValue::builder()
                .value("26")
                .display("26")
                .build(),
        ])
        .build();
}

fn main() {
    let args = AppArgs::parse();

    if !args.extcap.capture {
        if let Some(_filter) = args.extcap.extcap_capture_filter {
            std::process::exit(0);
        }
    }

    match args.extcap.run().unwrap() {
        ExtcapStep::Interfaces(interfaces_step) => {
            interfaces_step.list_interfaces(&METADATA, &[&*WIFI_CAPTURE_INTERFACE], &[]);
        }
        ExtcapStep::Dlts(dlts_step) => {
            dlts_step
                .print_from_interfaces(&[&*WIFI_CAPTURE_INTERFACE])
                .unwrap();
        }
        ExtcapStep::Config(config_step) => {
            config_step.list_configs(&[&*CONFIG_SERIALPORT, &*CONFIG_CHANNEL])
        }
        ExtcapStep::ReloadConfig(_reload_config_step) => {
            panic!("Unsupported operation");
        }
        ExtcapStep::Capture(capture_step) => {
            let (data_link, prefix) = (DataLink::IEEE802_15_4, "@RAW [");

            let mut controls = (
                capture_step.spawn_channel_control_reader(),
                capture_step.new_control_sender(),
            );

            if let (Some(control_reader), Some(_control_sender)) = &mut controls {
                let packet = control_reader.read_packet().unwrap();
                assert_eq!(packet.command, ControlCommand::Initialized);
            }

            let pcap_header = PcapHeader {
                datalink: data_link,
                endianness: pcap_file::Endianness::Big,
                ..Default::default()
            };
            let mut pcap_writer = PcapWriter::with_header(capture_step.fifo, pcap_header).unwrap();

            let channel = args.channel.clone();

            let serialport = if args.serialport.is_empty() {
                let ports = serialport::available_ports().unwrap();
                if ports.len() != 1 {
                    panic!("There are more or less than one serial ports. Don't know which one to use.");
                }
                ports[0].port_name.clone()
            } else {
                args.serialport.clone()
            };

            let mut port = serialport::new(serialport, 115_200)
                .timeout(Duration::from_millis(100))
                .open()
                .expect("Failed to open port");

            // drain the input ... just to be on the safe side
            while let Ok(len) = port.read(&mut [0u8; 128]) {
                if len == 0 {
                    break;
                }
            }

            // maybe reset the target
            port.write_all(&[b'r']).unwrap();

            // wait for target to boot up
            std::thread::sleep(Duration::from_millis(1000));

            // configure channel
            port.write_all(format!("{:02}", (channel).parse::<u16>().unwrap()).as_bytes())
                .unwrap();

            let mut buf_read = BufReader::new(port);

            let mut packet = Vec::<u8>::new();
            let mut line = String::new();

            loop {
                if let (Some(control_reader), Some(control_sender)) = &mut controls {
                    if let Some(control_packet) = control_reader.try_read_packet() {
                        handle_control_packet(&control_packet, control_sender).unwrap();
                    }
                }

                packet.clear();
                line.clear();
                if let Ok(len) = buf_read.read_line(&mut line) {
                    if len > 0 {
                        if line.contains(prefix) {
                            if !line.contains(']') {
                                panic!("Unexpected {}", line);
                            }

                            let start = line.find(prefix).unwrap() + prefix.len();
                            let end = line.find(']').unwrap();
                            let line = line[start..end].to_string();
                            for hex in line.split(", ") {
                                let byte = u8::from_str_radix(hex, 16).unwrap();
                                packet.push(byte);
                            }
                        }

                        if !packet.is_empty() {
                            let len = packet[0] - 2;
                            let crc = crc(&packet[1..][..len as usize]);
                            packet.insert(1 + (len as usize), crc[0]);
                            packet.insert(1 + (len as usize) + 1, crc[1]);

                            pcap_writer
                                .write_packet(&PcapPacket::new(
                                    SystemTime::now().duration_since(UNIX_EPOCH).unwrap(),
                                    (len as u32) + 2,
                                    &packet[1..][..(len + 2) as usize],
                                ))
                                .unwrap();
                        }
                        stdout().flush().unwrap();
                    }
                }
            }
        }
    };
}

fn crc(buf: &[u8]) -> [u8; 2] {
    let mut c: u16 = 0;
    for &i in buf {
        for k in 0..8 {
            let b = ((i & (1 << k)) != 0) ^ ((c & 1) != 0);
            c >>= 1;
            if b {
                c ^= 1 << 15;
                c ^= 1 << 10;
                c ^= 1 << 3;
            }
        }
    }
    [((c & 0xFF) as u8), ((c >> 8) as u8)]
}

fn handle_control_packet(
    _control_packet: &ControlPacket<'_>,
    _control_sender: &mut ExtcapControlSender,
) -> Result<(), ()> {
    // currently nothing to do here
    Ok(())
}
