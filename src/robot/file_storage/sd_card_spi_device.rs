use defmt::{error, trace};
use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    spi::{ErrorType, Operation, SpiBus, SpiDevice},
};

#[derive(Debug)]
pub enum SDCardSPIDeviceError {
    Spi,
    Pin,
    Delay,
    ChipSelect, // Add other error variants as needed
}

impl embedded_hal::spi::Error for SDCardSPIDeviceError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match self {
            SDCardSPIDeviceError::Spi => embedded_hal::spi::ErrorKind::Other,
            SDCardSPIDeviceError::Pin => embedded_hal::spi::ErrorKind::Other,
            SDCardSPIDeviceError::Delay => embedded_hal::spi::ErrorKind::Other,
            SDCardSPIDeviceError::ChipSelect => embedded_hal::spi::ErrorKind::ChipSelectFault,
            // Map other error variants as needed
        }
    }
}

pub struct SDCardSPIDevice<SPI, CS, DELAY>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    DELAY: DelayNs,
{
    pub bus: SPI,
    cs: CS,
    delay: DELAY,
}

impl<SPI, CS, DELAY> ErrorType for SDCardSPIDevice<SPI, CS, DELAY>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    DELAY: DelayNs,
{
    type Error = SDCardSPIDeviceError;
}

impl<SPI, CS, DELAY> SpiDevice<u8> for SDCardSPIDevice<SPI, CS, DELAY>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    DELAY: DelayNs,
{
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        trace!(
            "SDCardSPIDevice::transaction - operations.len = {}",
            operations.len()
        );
        // Implement the SPI transaction here
        if self.cs.set_low().is_err() {
            return Err(SDCardSPIDeviceError::ChipSelect);
        }
        for operation in operations {
            if let Err(_e) = match operation {
                Operation::Write(data) => {
                    trace!(
                        "SDCardSPIDevice::transaction - write data.len = {}",
                        data.len()
                    );
                    self.bus.write(data)
                }
                Operation::Transfer(read_buf, write_buf) => {
                    trace!("SDCardSPIDevice::transaction - transfer read_buf.len = {}, write_buf.len = {}", read_buf.len(), write_buf.len());
                    self.bus.transfer(read_buf, write_buf)
                }
                Operation::Read(data) => {
                    trace!(
                        "SDCardSPIDevice::transaction - read data.len = {}",
                        data.len()
                    );
                    self.bus.read(data)
                }
                Operation::TransferInPlace(data) => {
                    trace!(
                        "SDCardSPIDevice::transaction - transfer_in_place data.len = {}",
                        data.len()
                    );
                    self.bus.transfer_in_place(data)
                }
                Operation::DelayNs(time) => {
                    trace!("SDCardSPIDevice::transaction - delay_ns time = {}", time);
                    self.delay.delay_ns(*time);
                    Ok(())
                }
            } {
                // iff an error accurs, deassert CS pin and return error
                self.cs.set_high().ok();
                error!("SDCardSPIDevice::transaction - error");
                return Err(SDCardSPIDeviceError::Spi);
            }
        }
        if self.bus.flush().is_err() {
            self.cs.set_high().ok();
            error!("SDCardSPIDevice::transaction - flush error");
            return Err(SDCardSPIDeviceError::Spi);
        }
        if self.cs.set_high().is_err() {
            error!("SDCardSPIDevice::transaction - chip select error");
            return Err(SDCardSPIDeviceError::ChipSelect);
        }
        Ok(())
    }
}

impl<SPI, CS, DELAY> SDCardSPIDevice<SPI, CS, DELAY>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    DELAY: DelayNs,
{
    pub fn new(bus: SPI, cs: CS, delay: DELAY) -> Self {
        Self { bus, cs, delay }
    }
}
