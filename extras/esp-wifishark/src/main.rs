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
    config::StringConfig,
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
}

lazy_static! {
    static ref METADATA: Metadata = Metadata {
        help_url: "http://github.com/esp-rs/esp-wifi".into(),
        display_description: "esp-wifi".into(),
        ..r_extcap::cargo_metadata!()
    };
    static ref WIFI_CAPTURE_INTERFACE: Interface = Interface {
        value: "wifi".into(),
        display: "esp-wifi Ethernet capture".into(),
        dlt: Dlt {
            data_link_type: DataLink::USER0,
            name: "USER0".into(),
            display: "Ethernet".into(),
        },
    };
    static ref WIFI_RAW_CAPTURE_INTERFACE: Interface = Interface {
        value: "wifi_rfmon".into(),
        display: "esp-wifi IEEE802.11 capture".into(),
        dlt: Dlt {
            data_link_type: DataLink::USER1,
            name: "USER1".into(),
            display: "IEEE802.11".into(),
        },
    };
    static ref BT_CAPTURE_INTERFACE: Interface = Interface {
        value: "bt".into(),
        display: "esp-wifi HCI capture".into(),
        dlt: Dlt {
            data_link_type: DataLink::USER2,
            name: "USER2".into(),
            display: "HCI".into(),
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
            interfaces_step.list_interfaces(
                &METADATA,
                &[&*WIFI_CAPTURE_INTERFACE, &*WIFI_RAW_CAPTURE_INTERFACE, &*BT_CAPTURE_INTERFACE],
                &[],
            );
        }
        ExtcapStep::Dlts(dlts_step) => {
            dlts_step
                .print_from_interfaces(&[&*WIFI_CAPTURE_INTERFACE, &*WIFI_RAW_CAPTURE_INTERFACE, &*BT_CAPTURE_INTERFACE])
                .unwrap();
        }
        ExtcapStep::Config(config_step) => config_step.list_configs(&[&*CONFIG_SERIALPORT]),
        ExtcapStep::ReloadConfig(_reload_config_step) => {
            panic!("Unsupported operation");
        }
        ExtcapStep::Capture(capture_step) => {
            let (data_link, prefix) = if capture_step.interface == WIFI_CAPTURE_INTERFACE.value {
                (DataLink::ETHERNET, "@WIFIFRAME [")
            } else if capture_step.interface == WIFI_RAW_CAPTURE_INTERFACE.value {
                (DataLink::IEEE802_11, "@WIFIFRAME [")
            } else {
                (DataLink::BLUETOOTH_HCI_H4, "@HCIFRAME [")
            };

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

            let serialport = if args.serialport.is_empty() {
                let ports = serialport::available_ports().unwrap();
                if ports.len() != 1 {
                    panic!("There are more or less than one serial ports. Don't know which one to use.");
                }
                ports[0].port_name.clone()
            } else {
                args.serialport.clone()
            };

            let port = serialport::new(serialport, 115_200)
                .timeout(Duration::from_millis(100))
                .open()
                .expect("Failed to open port");
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
                            let start = line.find(prefix).unwrap() + prefix.len();
                            let end = line.find("]").unwrap();
                            let line = line[start..end].to_string();
                            for hex in line.split(", ") {
                                let byte = u8::from_str_radix(hex, 10).unwrap();
                                packet.push(byte);
                            }
                        }

                        if packet.len() > 0 {
                            pcap_writer
                                .write_packet(&PcapPacket::new(
                                    SystemTime::now().duration_since(UNIX_EPOCH).unwrap(),
                                    packet.len() as u32,
                                    &packet,
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

fn handle_control_packet(
    _control_packet: &ControlPacket<'_>,
    _control_sender: &mut ExtcapControlSender,
) -> Result<(), ()> {
    // currently nothing to do here
    Ok(())
}
