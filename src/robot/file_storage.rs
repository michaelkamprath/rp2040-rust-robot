#![allow(clippy::type_complexity)]
use alloc::{rc::Rc, string::String};
use core::cell::RefCell;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::OutputPin;
// Link in the embedded_sdmmc crate.
// The `SdMmcSpi` is used for block level access to the card.
// And the `VolumeManager` gives access to the FAT filesystem functions.
use embedded_sdmmc::{
    Directory, Mode, SdCard, TimeSource, Timestamp, Volume, VolumeIdx, VolumeManager,
};

// Get the file open mode enum:
use defmt::{debug, error, info};

/// A dummy timesource, which is mostly important for creating files.
#[derive(Default)]
pub struct DummyTimesource();

impl TimeSource for DummyTimesource {
    // In theory you could use the RTC of the rp2040 here, if you had
    // any external time synchronizing device.
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

pub struct FileStorage<SPI, CS, DELAY>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    CS: OutputPin,
    DELAY: DelayUs<u8>,
{
    volume_mgr: Option<Rc<RefCell<VolumeManager<SdCard<SPI, CS, DELAY>, DummyTimesource>>>>,
    volume: Option<Volume>,
    root_dir: Option<Directory>,
}

impl<SPI, CS, DELAY> FileStorage<SPI, CS, DELAY>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    CS: OutputPin,
    DELAY: DelayUs<u8>,
{
    pub fn new(spi: SPI, cs: CS, delay: DELAY) -> Self {
        info!("Initialize SPI SD/MMC data structures...");
        let sd_card = SdCard::new(spi, cs, delay);
        let mut volume_mgr = VolumeManager::new(sd_card, DummyTimesource::default());

        // check that we can read the volumn size
        match volume_mgr.device().num_bytes() {
            Ok(size) => {
                info!("SD card initialized. Volume size: {}", size);
            }
            Err(e) => {
                error!("Error reading volume size: {:?}", e);
                return Self {
                    volume_mgr: None,
                    volume: None,
                    root_dir: None,
                };
            }
        }

        debug!("Getting Volume 0...");
        let volume = match volume_mgr.open_volume(VolumeIdx(0)) {
            Ok(v) => v,
            Err(e) => {
                error!("Error getting volume 0: {}", defmt::Debug2Format(&e));
                return Self {
                    volume_mgr: None,
                    volume: None,
                    root_dir: None,
                };
            }
        };
        // After we have the volume (partition) of the drive we got to open the
        // root directory:
        debug!("Opening root dir...");
        let root_dir = match volume_mgr.open_root_dir(volume) {
            Ok(dir) => dir,
            Err(e) => {
                error!("Error opening root dir: {}", defmt::Debug2Format(&e));
                return Self {
                    volume_mgr: None,
                    volume: None,
                    root_dir: None,
                };
            }
        };
        info!("Root directory opened!");

        Self {
            volume_mgr: Some(Rc::new(RefCell::new(volume_mgr))),
            volume: Some(volume),
            root_dir: Some(root_dir),
        }
    }

    pub fn volume_size(&mut self) -> Option<u64> {
        match self.volume_mgr {
            None => None,
            Some(ref mut volume_mgr) => match volume_mgr.as_ref().borrow_mut().device().num_bytes()
            {
                Ok(size) => {
                    debug!("Volume size: {}", size);
                    Some(size)
                }
                Err(e) => {
                    error!("Error reading volume size: {:?}", e);
                    None
                }
            },
        }
    }

    pub fn root_dir(&self) -> Option<Directory> {
        self.root_dir
    }

    pub fn spi<T, F>(&self, func: F) -> Option<T>
    where
        F: FnOnce(&mut SPI) -> T,
    {
        Some(
            self.volume_mgr
                .as_ref()?
                .as_ref()
                .borrow_mut()
                .device()
                .spi(func),
        )
    }

    pub fn open_file_in_dir(
        &self,
        filename: &str,
        directory: Directory,
        mode: Mode,
    ) -> Option<SDFile<SPI, CS, DELAY>> {
        let file = match self
            .volume_mgr
            .as_ref()?
            .as_ref()
            .borrow_mut()
            .open_file_in_dir(directory, filename, mode)
        {
            Ok(f) => f,
            Err(e) => {
                error!("Error opening file '{}': {:?}", filename, e);
                return None;
            }
        };
        Some(SDFile::new(
            filename,
            file,
            self.volume_mgr.as_ref().unwrap().clone(),
        ))
    }
}

impl<SPI, CS, DELAY> Drop for FileStorage<SPI, CS, DELAY>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    CS: OutputPin,
    DELAY: DelayUs<u8>,
{
    fn drop(&mut self) {
        info!("FileStorage dropped");
        match &mut self.volume_mgr {
            Some(volume_mgr) => {
                if let Err(e) = volume_mgr
                    .as_ref()
                    .borrow_mut()
                    .close_dir(self.root_dir.unwrap())
                {
                    error!("Error closing root dir: {:?}", e);
                    return;
                }
                if let Err(e) = volume_mgr
                    .as_ref()
                    .borrow_mut()
                    .close_volume(self.volume.unwrap())
                {
                    error!("Error closing volume: {:?}", e);
                }
            }
            None => {}
        }
    }
}

pub struct SDFile<SPI, CS, DELAY>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    CS: OutputPin,
    DELAY: DelayUs<u8>,
{
    filename: String,
    file: embedded_sdmmc::File,
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
        file: embedded_sdmmc::File,
        volume_manager: Rc<RefCell<VolumeManager<SdCard<SPI, CS, DELAY>, DummyTimesource>>>,
    ) -> Self {
        Self {
            filename: String::from(filename),
            file,
            volume_manager,
        }
    }

    pub fn read<E: core::fmt::Debug>(
        &mut self,
        _buf: &mut [u8],
    ) -> Result<usize, embedded_sdmmc::Error<E>> {
        Ok(0)
    }

    pub fn write<E: core::fmt::Debug>(
        &mut self,
        _buf: &[u8],
    ) -> Result<usize, embedded_sdmmc::Error<E>> {
        Ok(0)
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
        if let Err(e) = self
            .volume_manager
            .as_ref()
            .borrow_mut()
            .close_file(self.file)
        {
            error!(
                "Error closing file '{}' at drop: {:?}",
                self.filename.as_str(),
                e
            );
        } else {
            debug!("File '{}' dropped", self.filename.as_str());
        }
    }
}
