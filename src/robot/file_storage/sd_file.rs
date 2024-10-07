use super::DummyTimesource;
use alloc::rc::Rc;
use alloc::string::String;
use core::cell::RefCell;
use defmt::{debug, error};
use embedded_hal::{delay::DelayNs, spi::SpiDevice};
use embedded_sdmmc::{Mode, RawDirectory, SdCard, SdCardError, VolumeManager};

pub struct SDFile<SPI_DEV, DELAY>
where
    SPI_DEV: SpiDevice<u8>,
    DELAY: DelayNs,
{
    filename: String,
    directory: RawDirectory,
    mode: Mode,
    volume_manager: Rc<RefCell<VolumeManager<SdCard<SPI_DEV, DELAY>, DummyTimesource>>>,
}

impl<SPI_DEV, DELAY> SDFile<SPI_DEV, DELAY>
where
    SPI_DEV: SpiDevice<u8>,
    DELAY: DelayNs,
{
    pub fn new(
        filename: &str,
        directory: RawDirectory,
        mode: Mode,
        volume_manager: Rc<RefCell<VolumeManager<SdCard<SPI_DEV, DELAY>, DummyTimesource>>>,
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
        if let Err(e) = self.volume_manager.borrow_mut().close_file(file) {
            error!("Error closing file '{}': {:?}", self.filename.as_str(), e);
            return Err(e);
        }
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
        self.volume_manager.borrow_mut().write(file, buf)?;
        self.volume_manager.borrow_mut().close_file(file)?;
        Ok(buf.len())
    }
}

impl<SPI_DEV, DELAY> Drop for SDFile<SPI_DEV, DELAY>
where
    SPI_DEV: SpiDevice<u8>,
    DELAY: DelayNs,
{
    fn drop(&mut self) {
        if let Err(e) = self.volume_manager.borrow_mut().close_dir(self.directory) {
            error!("Error closing directory: {:?}", e);
        }
        debug!("SDFile '{}' dropped", self.filename.as_str());
    }
}
