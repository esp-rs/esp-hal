//! Espressif provisioning GATT service (Read+Write+Notify + user descriptions).

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use heapless::Vec as HVec;
use trouble_host::{
    attribute::{AttributeTable, Characteristic, CharacteristicProp, PermissionLevel, Service},
    prelude::*,
    types::uuid::Uuid,
};

use crate::{PROTO_VER_JSON, PROV_SERVICE_UUID_128, SESSION_RESPONSE_SEC0};

const USER_DESC_UUID: Uuid = Uuid::Uuid16(0x2901u16.to_le_bytes());
const DEFAULT_GET_STATUS: [u8; 2] = [0x08, 0x00];
const RWN: [CharacteristicProp; 3] = [
    CharacteristicProp::Read,
    CharacteristicProp::Write,
    CharacteristicProp::Notify,
];

const ATT_MAX: usize = 32;
const CONN_MAX: usize = 2;

fn prov_char_uuid(u16_uuid: u16) -> [u8; 16] {
    let mut u = PROV_SERVICE_UUID_128;
    u[12] = (u16_uuid & 0xFF) as u8;
    u[13] = (u16_uuid >> 8) as u8;
    u
}

/// Build the provisioning GATT attribute table and characteristic handles.
pub fn build_prov_gatt_server<'a>(
    session_buf: &'a mut [u8; 256],
    config_buf: &'a mut [u8; 256],
    ctrl_buf: &'a mut [u8; 32],
    scan_buf: &'a mut [u8; 512],
    proto_ver_buf: &'a mut [u8; 128],
) -> (
    AttributeServer<'a, NoopRawMutex, DefaultPacketPool, ATT_MAX, CONN_MAX>,
    Characteristic<HVec<u8, 256>>,
    Characteristic<HVec<u8, 256>>,
    Characteristic<HVec<u8, 512>>,
    Characteristic<HVec<u8, 128>>,
) {
    let mut table: AttributeTable<'a, NoopRawMutex, ATT_MAX> = AttributeTable::new();
    let service_uuid = Uuid::Uuid128(PROV_SERVICE_UUID_128);
    let mut svc = table.add_service(Service { uuid: service_uuid });

    let ctrl_initial = HVec::<u8, 32>::new();
    let mut ctrl_builder = svc.add_characteristic(
        Uuid::Uuid128(prov_char_uuid(0xFF4F)),
        RWN,
        ctrl_initial,
        &mut ctrl_buf[..32],
    );
    ctrl_builder.add_descriptor_ro(USER_DESC_UUID, PermissionLevel::Allowed, "prov-ctrl");
    let _ctrl = ctrl_builder.build();

    let scan_initial = HVec::<u8, 512>::new();
    let mut scan_builder = svc.add_characteristic(
        Uuid::Uuid128(prov_char_uuid(0xFF50)),
        RWN,
        scan_initial,
        &mut scan_buf[..],
    );
    scan_builder.add_descriptor_ro(USER_DESC_UUID, PermissionLevel::Allowed, "prov-scan");
    let scan_char = scan_builder.build();

    let mut session_initial = HVec::<u8, 256>::new();
    let _ = session_initial.extend_from_slice(&SESSION_RESPONSE_SEC0);
    let mut session_builder = svc.add_characteristic(
        Uuid::Uuid128(prov_char_uuid(0xFF51)),
        RWN,
        session_initial,
        &mut session_buf[..],
    );
    session_builder.add_descriptor_ro(USER_DESC_UUID, PermissionLevel::Allowed, "prov-session");
    let session_char = session_builder.build();

    let mut config_initial = HVec::<u8, 256>::new();
    let _ = config_initial.extend_from_slice(&DEFAULT_GET_STATUS);
    let mut config_builder = svc.add_characteristic(
        Uuid::Uuid128(prov_char_uuid(0xFF52)),
        RWN,
        config_initial,
        &mut config_buf[..],
    );
    config_builder.add_descriptor_ro(USER_DESC_UUID, PermissionLevel::Allowed, "prov-config");
    let config_char = config_builder.build();

    let mut ver_initial = HVec::<u8, 128>::new();
    let _ = ver_initial.extend_from_slice(PROTO_VER_JSON);
    let mut ver_builder = svc.add_characteristic(
        Uuid::Uuid128(prov_char_uuid(0xFF53)),
        RWN,
        ver_initial,
        proto_ver_buf,
    );
    ver_builder.add_descriptor_ro(USER_DESC_UUID, PermissionLevel::Allowed, "proto-ver");
    let ver_char = ver_builder.build();

    let _service_handle = svc.build();
    let server = AttributeServer::new(table);
    (server, session_char, config_char, scan_char, ver_char)
}
