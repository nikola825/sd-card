use core::fmt::Debug;

use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, big_endian};

use crate::{INTERFACE_CONDITON_3V3_SUPPORTED, INTERFACE_CONDITON_CHECK_PATTERN, SDError};

pub trait SDREsponse<
    SpiErrorType: embedded_hal_async::spi::Error,
    CsErrorType: embedded_hal::digital::Error,
>: FromBytes + KnownLayout + Sized + Default + IntoBytes
{
    fn validate(&self) -> Result<(), SDError<SpiErrorType, CsErrorType>>;
}

#[derive(Clone, Copy, FromBytes, Immutable, KnownLayout, IntoBytes)]
#[repr(C)]
pub struct R1 {
    pub r1: u8,
}

impl Default for R1 {
    fn default() -> Self {
        Self { r1: 0xff }
    }
}

impl Debug for R1 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if !self.is_valid() {
            write!(f, "R1 {{ invalid(0x{:02x}) }}", self.r1)
        } else if !self.is_error_free() {
            write!(f, "R1 {{ error(0x{:02x}) }}", self.r1)
        } else if self.is_powered_up() {
            write!(f, "R1 {{ powered up }}")
        } else {
            write!(f, "R1 {{ idle }}")
        }
    }
}

impl<SpiErrorType: embedded_hal_async::spi::Error, CsErrorType: embedded_hal::digital::Error>
    SDREsponse<SpiErrorType, CsErrorType> for R1
{
    fn validate(&self) -> Result<(), SDError<SpiErrorType, CsErrorType>> {
        if !self.is_valid() {
            Err(SDError::R1NotValid(*self))
        } else if !self.is_error_free() {
            Err(SDError::R1NotErrorFree(*self))
        } else {
            Ok(())
        }
    }
}

impl R1 {
    pub fn is_valid(&self) -> bool {
        self.r1 & 0x80 == 0
    }

    pub fn is_powered_up(&self) -> bool {
        self.r1 == 0
    }

    pub fn is_error_free(&self) -> bool {
        self.r1 <= 1
    }

    pub fn expect_powered_up<
        SpiErrorType: embedded_hal_async::spi::Error,
        CsErrorType: embedded_hal::digital::Error,
    >(
        &self,
    ) -> Result<(), SDError<SpiErrorType, CsErrorType>> {
        self.validate()?;
        if !self.is_powered_up() {
            Err(SDError::R1NotPoweredUp(*self))
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Copy, Default, FromBytes, Immutable, KnownLayout, IntoBytes)]
#[repr(C)]
pub struct R7 {
    pub r1: R1,
    pub _command_version: u8,
    pub _reserved: u8,
    pub voltage_accepted: u8,
    pub echo: u8,
}

impl Debug for R7 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.voltage_accepted == INTERFACE_CONDITON_3V3_SUPPORTED {
            write!(
                f,
                "R7 {{ r1:{{ {:?} }}; 3v3_accepted:yes; echo:0x{:02x} }}",
                self.r1, self.echo
            )
        } else {
            write!(
                f,
                "R7 {{ r1:{{ {:?} }}; 3v3_accepted:no(0x{:02x}); echo:0x{:02x} }}",
                self.r1, self.voltage_accepted, self.echo
            )
        }
    }
}

impl<SpiErrorType: embedded_hal_async::spi::Error, CsErrorType: embedded_hal::digital::Error>
    SDREsponse<SpiErrorType, CsErrorType> for R7
{
    fn validate(&self) -> Result<(), SDError<SpiErrorType, CsErrorType>> {
        self.r1.validate()?;
        if self.echo != INTERFACE_CONDITON_CHECK_PATTERN {
            Err(SDError::InterfaceConditionBadCheck(*self))
        } else if self.voltage_accepted != INTERFACE_CONDITON_3V3_SUPPORTED {
            Err(SDError::InterfaceConditionNo3v3Support(*self))
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Copy, Default, FromBytes, Immutable, KnownLayout, IntoBytes)]
#[repr(C)]
pub struct OCR {
    pub status_data: u8,
    pub voltage_data: u8,
    pub low_voltage_data: big_endian::U16,
}

impl OCR {
    pub fn is_powered_up(&self) -> bool {
        self.status_data & 0x80 == 0x80
    }

    pub fn is_sdhc(&self) -> bool {
        self.status_data & 0x40 == 0x40
    }

    pub fn expect_powered_up_and_sdhc<
        SpiErrorType: embedded_hal_async::spi::Error,
        CsErrorType: embedded_hal::digital::Error,
    >(
        &self,
    ) -> Result<(), SDError<SpiErrorType, CsErrorType>> {
        if !self.is_powered_up() {
            Err(SDError::OCRNotPoweredUp(*self))
        } else if !self.is_sdhc() {
            Err(SDError::OCRNotSDHC(*self))
        } else {
            Ok(())
        }
    }
}

impl Debug for OCR {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.is_powered_up() {
            if self.is_sdhc() {
                write!(f, "OCR {{ powered_up:yes; sdhc:yes }}",)
            } else {
                write!(
                    f,
                    "OCR {{ powered_up: yes; sdhc:no(0x{:02x}) }}",
                    self.status_data
                )
            }
        } else {
            write!(f, "OCR {{ powered_up:no(0x{:02x}) }}", self.status_data)
        }
    }
}

#[derive(Clone, Copy, Debug, Default, FromBytes, Immutable, KnownLayout, IntoBytes)]
#[repr(C)]
pub struct R3 {
    pub r1: R1,
    pub ocr: OCR,
}

impl<SpiErrorType: embedded_hal_async::spi::Error, CsErrorType: embedded_hal::digital::Error>
    SDREsponse<SpiErrorType, CsErrorType> for R3
{
    fn validate(&self) -> Result<(), SDError<SpiErrorType, CsErrorType>> {
        self.r1.validate()
    }
}

#[derive(Clone, Copy, Debug, FromBytes, Immutable, KnownLayout, IntoBytes)]
#[repr(C)]
pub struct ReadBlock {
    pub start_byte: u8,
    pub data: [u8; 512],
    pub crc: [u8; 2],
}

impl Default for ReadBlock {
    fn default() -> Self {
        Self {
            start_byte: Default::default(),
            data: [0; 512],
            crc: Default::default(),
        }
    }
}

impl<SpiErrorType: embedded_hal_async::spi::Error, CsErrorType: embedded_hal::digital::Error>
    SDREsponse<SpiErrorType, CsErrorType> for ReadBlock
{
    fn validate(&self) -> Result<(), SDError<SpiErrorType, CsErrorType>> {
        if self.start_byte != 0xfe {
            Err(SDError::StartByteNotFe(self.start_byte))
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Copy, Debug, FromBytes, Immutable, KnownLayout, IntoBytes, Default)]
#[repr(C)]
pub struct WriteStartedResponse {
    pub write_status: u8,
}

impl<SpiErrorType: embedded_hal_async::spi::Error, CsErrorType: embedded_hal::digital::Error>
    SDREsponse<SpiErrorType, CsErrorType> for WriteStartedResponse
{
    fn validate(&self) -> Result<(), SDError<SpiErrorType, CsErrorType>> {
        const SUCCESS_MASK: u8 = 0b00000101;
        if self.write_status & SUCCESS_MASK != SUCCESS_MASK {
            Err(SDError::WriteStartStatusInvalid(*self))
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Copy, Debug, FromBytes, Immutable, KnownLayout, IntoBytes, Default)]
#[repr(C)]
pub struct WriteEndedResponse {
    pub write_status: u8,
}

impl<SpiErrorType: embedded_hal_async::spi::Error, CsErrorType: embedded_hal::digital::Error>
    SDREsponse<SpiErrorType, CsErrorType> for WriteEndedResponse
{
    fn validate(&self) -> Result<(), SDError<SpiErrorType, CsErrorType>> {
        if self.write_status == 0 {
            Err(SDError::WriteEndStatusInvalid(*self))
        } else {
            Ok(())
        }
    }
}
