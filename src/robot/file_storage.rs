#![allow(clippy::type_complexity)]
pub mod logger;
pub mod sd_card_spi_device;
pub mod sd_file;

use alloc::{rc::Rc, string::ToString, vec::Vec};
use core::cell::RefCell;
use defmt::{debug, error, info, trace};
use embedded_hal::{delay::DelayNs, spi::SpiDevice};
use embedded_sdmmc::{
    DirEntry, Mode, RawDirectory, RawVolume, SdCard, SdCardError, TimeSource, Timestamp, VolumeIdx,
    VolumeManager,
};
use sd_file::SDFile;

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

pub struct FileStorage<SPI_DEV, DELAY>
where
    SPI_DEV: SpiDevice<u8>,
    DELAY: DelayNs,
{
    volume_mgr: Option<Rc<RefCell<VolumeManager<SdCard<SPI_DEV, DELAY>, DummyTimesource>>>>,
    volume: Option<RawVolume>,
    root_dir: Option<RawDirectory>,
}

impl<SPI_DEV, DELAY> FileStorage<SPI_DEV, DELAY>
where
    SPI_DEV: SpiDevice<u8>,
    DELAY: DelayNs,
{
    pub fn new(spi: SPI_DEV, delay: DELAY) -> Self {
        info!("Initialize SPI SD/MMC data structures...");
        let sd_card = SdCard::new(spi, delay);
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

        trace!("Getting Volume 0...");
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
        let raw_volume = volume.to_raw_volume();
        let root_dir = match volume_mgr.open_root_dir(raw_volume) {
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
            volume: Some(raw_volume),
            root_dir: Some(root_dir),
        }
    }

    pub fn volume_manager(
        &self,
    ) -> Option<Rc<RefCell<VolumeManager<SdCard<SPI_DEV, DELAY>, DummyTimesource>>>> {
        self.volume_mgr.clone()
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

    pub fn root_dir(&self) -> Option<RawDirectory> {
        self.root_dir
    }

    pub fn open_directory(
        &self,
        parent: RawDirectory,
        name: &str,
    ) -> Result<RawDirectory, embedded_sdmmc::Error<SdCardError>> {
        match self
            .volume_mgr
            .as_ref()
            .unwrap()
            .borrow_mut()
            .open_dir(parent, name)
        {
            Ok(dir) => Ok(dir),
            Err(e) => {
                error!("Error opening directory '{}': {:?}", name, e);
                Err(e)
            }
        }
    }

    pub fn spi<T, F>(&self, func: F) -> Option<T>
    where
        F: FnOnce(&mut SPI_DEV) -> T,
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
        &mut self,
        filename: &str,
        directory: RawDirectory,
        mode: Mode,
    ) -> Result<SDFile<SPI_DEV, DELAY>, embedded_sdmmc::Error<SdCardError>> {
        Ok(SDFile::new(
            filename,
            directory,
            mode,
            self.volume_mgr.as_ref().unwrap().clone(),
        ))
    }

    pub fn iterate_dir<F>(
        &self,
        dir: RawDirectory,
        func: F,
    ) -> Result<(), embedded_sdmmc::Error<SdCardError>>
    where
        F: FnMut(&DirEntry),
    {
        self.volume_mgr
            .as_ref()
            .unwrap()
            .borrow_mut()
            .iterate_dir(dir, func)
    }

    pub fn list_files_in_dir_with_ext(
        &self,
        parent_dir: RawDirectory,
        target_dir: &str,
        ext: &str,
    ) -> Result<Vec<DirEntry>, embedded_sdmmc::Error<SdCardError>> {
        let mut files: Vec<DirEntry> = Vec::new();
        let dir = self.open_directory(parent_dir, target_dir)?;
        self.iterate_dir(dir, |entry| {
            if entry.name.to_string().ends_with(ext) {
                debug!("Found path file: {:?}", entry.name.to_string().as_str());
                files.push(entry.clone());
            }
        })?;
        self.volume_mgr
            .as_ref()
            .unwrap()
            .borrow_mut()
            .close_dir(dir)?;
        Ok(files)
    }
}

impl<SPI_DEV, DELAY> Drop for FileStorage<SPI_DEV, DELAY>
where
    SPI_DEV: SpiDevice,
    DELAY: DelayNs,
{
    fn drop(&mut self) {
        info!("FileStorage dropped");
        if let Some(volume_mgr) = &mut self.volume_mgr {
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
    }
}
