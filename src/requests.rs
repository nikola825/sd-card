use zerocopy::{Immutable, IntoBytes, TryFromBytes, big_endian};

use crate::{INTERFACE_CONDITON_3V3_SUPPORTED, INTERFACE_CONDITON_CHECK_PATTERN};

#[derive(Clone, Copy, Debug, IntoBytes, Immutable)]
#[repr(C)]
pub struct GetInterfaceConditionArgument {
    reserved: big_endian::U16,
    supply_voltage: u8,
    check_pattern: u8,
}

impl Default for GetInterfaceConditionArgument {
    fn default() -> Self {
        Self {
            reserved: 0.into(),
            supply_voltage: INTERFACE_CONDITON_3V3_SUPPORTED,
            check_pattern: INTERFACE_CONDITON_CHECK_PATTERN,
        }
    }
}

impl GetInterfaceConditionArgument {
    pub fn as_number(&self) -> u32 {
        big_endian::U32::try_read_from_bytes(self.as_bytes())
            .unwrap()
            .into()
    }
}

#[derive(Clone, Copy, Debug, IntoBytes, Immutable)]
#[repr(C)]
pub struct SendOpConditionArgument {
    hcs_support: u8,
    reserved: [u8; 3],
}

impl Default for SendOpConditionArgument {
    fn default() -> Self {
        Self {
            hcs_support: 0x40,
            reserved: [0; 3],
        }
    }
}

impl SendOpConditionArgument {
    pub fn as_number(&self) -> u32 {
        big_endian::U32::try_read_from_bytes(self.as_bytes())
            .unwrap()
            .into()
    }
}

#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum SDCommand {
    GO_TO_IDLE = 0,
    SEND_IF_COND = 8,
    READ_SINGLE_BLOCK = 17,
    WRITE_BLOCK = 24,
    AP_SD_SEND_OP_COND = 41,
    AP_CMD = 55,
    READ_OCR = 58,
}

#[derive(Clone, Copy, Default, IntoBytes, Immutable)]
#[repr(C)]
pub struct SDCommandPacket {
    command: u8,
    argument: big_endian::U32,
    crc: u8,
}

impl SDCommandPacket {
    pub fn new(command: SDCommand, argument: u32) -> Self {
        let command = (command as u8) | 0x40;
        let crc = 0;
        let mut me = Self {
            command,
            argument: argument.into(),
            crc,
        };

        me.fill_crc();

        me
    }

    fn fill_crc(&mut self) {
        let bytes = self.as_bytes();

        let mut crc: u8 = 0;
        for byte in &bytes[..5] {
            crc ^= *byte;
            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc ^= 0x89;
                }
                crc <<= 1;
            }
        }
        self.crc = crc | 1;
    }
}
