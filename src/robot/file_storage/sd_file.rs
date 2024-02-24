use super::DummyTimesource;
use alloc::rc::Rc;
use alloc::string::String;
use core::cell::RefCell;
use defmt::{debug, error};
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::OutputPin;
use embedded_sdmmc::{Directory, Mode, SdCard, SdCardError, VolumeManager};

pub struct SDFile<SPI, CS, DELAY>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    CS: OutputPin,
    DELAY: DelayUs<u8>,
{
    filename: String,
    directory: Directory,
    mode: Mode,
    volume_manager: Rc<RefCell<VolumeManager<SdCard<SPI, CS, DELAY>, DummyTimesource>>>,
}

impl<SPI, CS, DELAY> SDFile<SPI, CS, DELAY>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    CS: OutputPin,
    DELAY: DelayUs<u8>,
{
    pub fn new(
        filename: &str,
        directory: Directory,
        mode: Mode,
        volume_manager: Rc<RefCell<VolumeManager<SdCard<SPI, CS, DELAY>, DummyTimesource>>>,
    ) -> Self {
        Self {
            filename: String::from(filename),
            directory,
            mode,
            volume_manager,
        }
    }

    pub fn init(&mut self) -> Result<(), embedded_sdmmc::Error<SdCardError>> {
        // first open the file per the passed mode
        let file = match self.volume_manager.borrow_mut().open_file_in_dir(
            self.directory,
            self.filename.as_str(),
            self.mode,
        ) {
            Ok(f) => {
                debug!(
                    "Opened file '{}' with mode {}",
                    self.filename.as_str(),
                    self.mode
                );
                f
            }
            Err(e) => {
                error!("Error opening file '{}': {:?}", self.filename.as_str(), e);
                return Err(e);
            }
        };
        // then close the file
        self.volume_manager.borrow_mut().close_file(file).unwrap();
        Ok(())
    }

    pub fn length(&self) -> Result<u32, embedded_sdmmc::Error<SdCardError>> {
        let file = self.volume_manager.borrow_mut().open_file_in_dir(
            self.directory,
            self.filename.as_str(),
            Mode::ReadOnly,
        )?;
        let length = self.volume_manager.borrow().file_length(file)?;
        self.volume_manager.borrow_mut().close_file(file)?;
        Ok(length)
    }

    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, embedded_sdmmc::Error<SdCardError>> {
        let file = self.volume_manager.borrow_mut().open_file_in_dir(
            self.directory,
            self.filename.as_str(),
            Mode::ReadOnly,
        )?;
        let read_size = self.volume_manager.borrow_mut().read(file, buf)?;
        self.volume_manager.borrow_mut().close_file(file)?;
        Ok(read_size)
    }

    pub fn write(&mut self, buf: &[u8]) -> Result<usize, embedded_sdmmc::Error<SdCardError>> {
        if self.mode == Mode::ReadOnly {
            return Err(embedded_sdmmc::Error::ReadOnly);
        }
        let file = self.volume_manager.borrow_mut().open_file_in_dir(
            self.directory,
            self.filename.as_str(),
            Mode::ReadWriteAppend,
        )?;
        let write_size = self.volume_manager.borrow_mut().write(file, buf)?;
        self.volume_manager.borrow_mut().close_file(file)?;
        Ok(write_size)
    }
}

impl<SPI, CS, DELAY> Drop for SDFile<SPI, CS, DELAY>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    CS: OutputPin,
    DELAY: DelayUs<u8>,
{
    fn drop(&mut self) {
        if let Err(e) = self.volume_manager.borrow_mut().close_dir(self.directory) {
            error!("Error closing directory: {:?}", e);
        }
        debug!("SDFile '{}' dropped", self.filename.as_str());
    }
}
