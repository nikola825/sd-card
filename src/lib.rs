#![no_std]

mod requests;
pub mod responses;

use core::time::Duration;

use embedded_hal::spi::SpiBus as SpiBusBlocking;
use embedded_hal_async::spi::SpiBus as SpiBusAsync;
use zerocopy::IntoBytes;

use crate::{
    requests::{
        GetInterfaceConditionArgument, SDCommand, SDCommandPacket, SendOpConditionArgument,
    },
    responses::{OCR, R1, R3, R7, ReadBlock, SDREsponse, WriteEndedResponse, WriteStartedResponse},
};

#[derive(Debug)]
pub enum SDError<
    SpiErrorType: embedded_hal_async::spi::Error,
    CsErrorType: embedded_hal::digital::Error,
> {
    R1NotValid(R1),
    R1NotErrorFree(R1),
    R1NotPoweredUp(R1),
    InterfaceConditionBadCheck(R7),
    InterfaceConditionNo3v3Support(R7),
    StartByteNotFe(u8),
    WriteStartStatusInvalid(WriteStartedResponse),
    WriteEndStatusInvalid(WriteEndedResponse),
    CsToggleError(CsErrorType),
    OCRNotPoweredUp(OCR),
    OCRNotSDHC(OCR),
    SpiError(SpiErrorType),
}

impl<SpiErrorType: embedded_hal_async::spi::Error, CsErrorType: embedded_hal::digital::Error>
    From<SpiErrorType> for SDError<SpiErrorType, CsErrorType>
{
    fn from(value: SpiErrorType) -> Self {
        Self::SpiError(value)
    }
}

impl<SpiErrorType: embedded_hal_async::spi::Error, CsErrorType: embedded_hal::digital::Error>
    SDError<SpiErrorType, CsErrorType>
{
    pub fn from_cs_error(cs_error: CsErrorType) -> Self {
        Self::CsToggleError(cs_error)
    }
}

const INTERFACE_CONDITON_3V3_SUPPORTED: u8 = 1;
const INTERFACE_CONDITON_CHECK_PATTERN: u8 = 0x42;

struct SPITransaction<
    'a,
    Bus: SpiBusAsync + SpiBusBlocking,
    CsPin: embedded_hal::digital::OutputPin,
> {
    spi: &'a mut Bus,
    cs: &'a mut CsPin,
}

impl<'a, Bus: SpiBusAsync + SpiBusBlocking, CsPin: embedded_hal::digital::OutputPin>
    SPITransaction<'a, Bus, CsPin>
{
    pub async fn new<Delay: embedded_hal_async::delay::DelayNs>(
        card: &'a mut SDCard<Bus, CsPin, Delay>,
    ) -> Result<Self, SDError<Bus::Error, CsPin::Error>> {
        let transaction = SPITransaction {
            spi: &mut card.inner_spi_do_not_use_without_transaction,
            cs: &mut card.inner_cs_do_not_use_without_transaction,
        };

        // Pull down CS and pull MOSI high, give the SDCard time to get ready
        SpiBusAsync::write(transaction.spi, &[0xffu8]).await?;
        transaction
            .cs
            .set_low()
            .map_err(SDError::<Bus::Error, CsPin::Error>::from_cs_error)?;
        SpiBusAsync::write(transaction.spi, &[0xffu8]).await?;

        Ok(transaction)
    }

    pub async fn read_buffer(&mut self, buf: &mut [u8]) -> Result<(), Bus::Error> {
        // Fill buffer with ones and output it while reading, to keep MOSI pulled up during transfer
        buf.fill(0xff);
        SpiBusAsync::transfer_in_place(self.spi, buf).await
    }

    pub async fn read_response<Response: SDREsponse<Bus::Error, CsPin::Error>>(
        &mut self,
        max_wait_cycles: u32,
        busy_indicator: u8,
    ) -> Result<Response, SDError<Bus::Error, CsPin::Error>> {
        let mut response = Response::default();
        let buffer = response.as_mut_bytes();
        for _ in 0..max_wait_cycles {
            self.read_buffer(&mut buffer[..1]).await?;
            if buffer[0] != busy_indicator {
                break;
            }
        }

        if buffer.len() > 1 {
            self.read_buffer(&mut buffer[1..]).await?;
        }

        response.validate()?;

        Ok(response)
    }

    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Bus::Error> {
        SpiBusAsync::write(self.spi, buffer).await
    }

    pub async fn send_command<Response: SDREsponse<Bus::Error, CsPin::Error>>(
        &mut self,
        command: SDCommandPacket,
    ) -> Result<Response, SDError<Bus::Error, CsPin::Error>> {
        self.write(command.as_bytes()).await?;

        let response: Response = self.read_response(10, 0xff).await?;

        response.validate()?;

        Ok(response)
    }
}

impl<'a, Bus: SpiBusAsync + SpiBusBlocking, CsPin: embedded_hal::digital::OutputPin> Drop
    for SPITransaction<'a, Bus, CsPin>
{
    fn drop(&mut self) {
        // Pull output and CS to high. Writing adds delay to give SD card time to settle
        SpiBusBlocking::write(self.spi, &[0xffu8]).unwrap();
        self.cs.set_high().unwrap();
        SpiBusBlocking::write(self.spi, &[0xffu8]).unwrap();
    }
}

pub struct SDCard<
    Bus: SpiBusAsync + SpiBusBlocking,
    CsPin: embedded_hal::digital::OutputPin,
    Delay: embedded_hal_async::delay::DelayNs,
> {
    inner_spi_do_not_use_without_transaction: Bus,
    inner_cs_do_not_use_without_transaction: CsPin,
    delay: Delay,
}

impl<
    Bus: SpiBusAsync + SpiBusBlocking,
    CsPin: embedded_hal::digital::OutputPin,
    Delay: embedded_hal_async::delay::DelayNs,
> SDCard<Bus, CsPin, Delay>
{
    pub fn new(spi: Bus, cs: CsPin, delay: Delay) -> Self {
        Self {
            inner_cs_do_not_use_without_transaction: cs,
            inner_spi_do_not_use_without_transaction: spi,
            delay,
        }
    }
    async fn begin_transaction<'a>(
        &'a mut self,
    ) -> Result<SPITransaction<'a, Bus, CsPin>, SDError<Bus::Error, CsPin::Error>> {
        SPITransaction::new(self).await
    }

    async fn reset(&mut self) -> Result<R1, SDError<Bus::Error, CsPin::Error>> {
        self.inner_cs_do_not_use_without_transaction
            .set_high()
            .map_err(SDError::<Bus::Error, CsPin::Error>::from_cs_error)?;

        SpiBusAsync::write(
            &mut self.inner_spi_do_not_use_without_transaction,
            &[0xffu8; 10],
        )
        .await?;

        let mut transaction = self.begin_transaction().await?;

        transaction
            .send_command(SDCommandPacket::new(SDCommand::GO_TO_IDLE, 0))
            .await
    }

    async fn get_interface_condition(&mut self) -> Result<R7, SDError<Bus::Error, CsPin::Error>> {
        let mut transaction = self.begin_transaction().await?;

        transaction
            .send_command(SDCommandPacket::new(
                SDCommand::SEND_IF_COND,
                GetInterfaceConditionArgument::default().as_number(),
            ))
            .await
    }

    async fn read_ocr(&mut self) -> Result<R3, SDError<Bus::Error, CsPin::Error>> {
        let mut transaction = self.begin_transaction().await?;

        transaction
            .send_command(SDCommandPacket::new(SDCommand::READ_OCR, 0))
            .await
    }

    async fn power_up(&mut self) -> Result<R1, SDError<Bus::Error, CsPin::Error>> {
        {
            let mut transaction = self.begin_transaction().await?;

            let _: R1 = transaction
                .send_command(SDCommandPacket::new(SDCommand::AP_CMD, 0))
                .await?;
        }
        {
            let mut transaction = self.begin_transaction().await?;

            transaction
                .send_command(SDCommandPacket::new(
                    SDCommand::AP_SD_SEND_OP_COND,
                    SendOpConditionArgument::default().as_number(),
                ))
                .await
        }
    }

    pub async fn initialize(&mut self) -> Result<(), SDError<Bus::Error, CsPin::Error>> {
        const POWER_UP_TIMEOUT: Duration = Duration::from_secs(1);
        const POWER_UP_BACKOFF: Duration = Duration::from_millis(5);

        self.reset().await?;

        self.get_interface_condition().await?;

        self.read_ocr().await?;

        let mut power_up_r1: R1 = R1::default();
        let power_up_retries = POWER_UP_TIMEOUT.as_micros() / POWER_UP_BACKOFF.as_micros();

        for _ in 0..power_up_retries {
            power_up_r1 = self.power_up().await?;
            if power_up_r1.is_powered_up() {
                break;
            }
            self.delay
                .delay_ms(POWER_UP_BACKOFF.as_millis() as u32)
                .await;
        }

        power_up_r1.expect_powered_up()?;

        let r3 = self.read_ocr().await?;

        r3.ocr.expect_powered_up_and_sdhc()?;

        Ok(())
    }

    pub async fn read_block(
        &mut self,
        addr: u32,
    ) -> Result<ReadBlock, SDError<Bus::Error, CsPin::Error>> {
        let mut transaction = self.begin_transaction().await?;

        let r1: R1 = transaction
            .send_command(SDCommandPacket::new(SDCommand::READ_SINGLE_BLOCK, addr))
            .await?;

        r1.expect_powered_up()?;

        transaction.read_response(100000, 0xff).await
    }

    pub async fn write_block(
        &mut self,
        addr: u32,
        block: &[u8; 512],
    ) -> Result<(), SDError<Bus::Error, CsPin::Error>> {
        let mut transaction = self.begin_transaction().await?;

        let r1: R1 = transaction
            .send_command(SDCommandPacket::new(SDCommand::WRITE_BLOCK, addr))
            .await?;

        r1.expect_powered_up()?;

        transaction.write(&[0xfe]).await?;
        transaction.write(block).await?;

        let _: WriteStartedResponse = transaction.read_response(100000, 0xff).await?;

        let _: WriteEndedResponse = transaction.read_response(100000, 0x00).await?;

        Ok(())
    }
}
