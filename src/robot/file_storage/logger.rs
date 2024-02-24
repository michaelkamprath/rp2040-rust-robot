use alloc::{
    format,
    string::{String, ToString},
    vec,
};
use defmt::{debug, error, info, warn};
use embedded_hal::{blocking::delay::DelayUs, digital::v2::OutputPin};
use embedded_sdmmc::SdCardError;

use super::{sd_file::SDFile, FileStorage};

//==============================================================================
// Directory and File namses
//
const LOG_DIR_NAME: &str = "log";
const LOG_INDEX_FILE_NAME: &str = "index.txt";

pub struct Logger<SPI, CS, DELAY, const BUFSIZE: usize>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    CS: OutputPin,
    DELAY: DelayUs<u8>,
{
    log_file: Option<SDFile<SPI, CS, DELAY>>,
    buffer: [u8; BUFSIZE],
    buffer_index: usize,
}

impl<SPI, CS, DELAY, const BUFSIZE: usize> Logger<SPI, CS, DELAY, BUFSIZE>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    CS: OutputPin,
    DELAY: DelayUs<u8>,
{
    pub fn new(log_file: Option<SDFile<SPI, CS, DELAY>>) -> Self {
        Self {
            log_file,
            buffer: [0u8; BUFSIZE],
            buffer_index: 0,
        }
    }

    /// Initializes the log file and log file index
    pub fn init_log_file(
        sdcard: &mut FileStorage<SPI, CS, DELAY>,
    ) -> Option<SDFile<SPI, CS, DELAY>> {
        // open log directory
        let root_dir = match sdcard.root_dir() {
            Some(dir) => dir,
            None => {
                warn!("Could not create log file. No logging will be done.");
                return None;
            }
        };
        let log_dir = match sdcard.open_directory(root_dir, LOG_DIR_NAME) {
            Some(dir) => dir,
            None => {
                error!("Error opening log dir");
                return None;
            }
        };

        // open log index file
        let log_index = match sdcard.open_file_in_dir(
            LOG_INDEX_FILE_NAME,
            log_dir,
            embedded_sdmmc::Mode::ReadOnly,
        ) {
            Ok(mut file) => {
                if let Err(_e) = file.init() {
                    warn!("Log index file does not exist. Starting log index at 0");
                    0
                } else {
                    // log index file exists. read contents.
                    let file_len = file.length().unwrap_or(0);
                    if file_len == 0 {
                        warn!("Log index file is empty. Starting log index at 0");
                        0
                    } else {
                        let mut buffer = vec![0u8; file_len as usize];
                        if let Err(e) = file.read(buffer.as_mut_slice()) {
                            error!("Error reading log index file: {:?}", e);
                            0
                        } else {
                            let contents = String::from_utf8(buffer).unwrap_or_default();
                            debug!("Log index file contents:\n{:?}", contents.as_str());
                            match contents.parse::<u32>() {
                                Ok(index) => index + 1,
                                Err(e) => {
                                    error!(
                                        "Error parsing log index file contents: {:?}",
                                        e.to_string().as_str()
                                    );
                                    0
                                }
                            }
                        }
                    }
                }
            }
            Err(_e) => {
                warn!("Error opening log index file contents. Starting log index at 0");
                0
            }
        };

        // write the new log index back to the file. Need to re-open the containing directory first.
        let log_dir = match sdcard.open_directory(root_dir, LOG_DIR_NAME) {
            Some(dir) => dir,
            None => {
                error!("Error opening log dir");
                return None;
            }
        };
        match sdcard.open_file_in_dir(
            LOG_INDEX_FILE_NAME,
            log_dir,
            embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
        ) {
            Ok(mut file) => {
                if let Err(e) = file.init() {
                    error!("Error initializing log index file: {:?}", e);
                }
                let contents = format!("{}", log_index);
                if let Err(e) = file.write(contents.as_bytes()) {
                    error!("Error writing log index file: {:?}", e);
                } else {
                    info!("Log index file updated to {}", log_index);
                }
            }
            Err(e) => {
                error!("Error opening log index file for writing: {:?}", e);
            }
        }
        info!("Log index = {}", log_index);

        // now creater the logger
        let log_filename = format!("log_{}.txt", log_index);
        let log_dir = match sdcard.open_directory(root_dir, LOG_DIR_NAME) {
            Some(dir) => dir,
            None => {
                error!("Error opening log dir");
                return None;
            }
        };
        let log_file = match sdcard.open_file_in_dir(
            log_filename.as_str(),
            log_dir,
            embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
        ) {
            Ok(mut file) => {
                if let Err(e) = file.init() {
                    error!("Error initializing log file: {:?}", e);
                    return None;
                }
                file
            }
            Err(e) => {
                error!("Error opening log file: {:?}", e);
                return None;
            }
        };
        debug!("Log file opened: {:?}", log_filename.as_str());
        Some(log_file)
    }

    pub fn write_to_buffer(&mut self, s: &str) -> Result<(), embedded_sdmmc::Error<SdCardError>> {
        let bytes = s.as_bytes();
        let mut idx: usize = 0;

        while bytes[idx..].len() > BUFSIZE - self.buffer_index {
            let remaining = BUFSIZE - self.buffer_index;
            self.buffer[self.buffer_index..BUFSIZE].copy_from_slice(&bytes[idx..idx + remaining]);
            self.buffer_index = BUFSIZE;
            idx += remaining;
            self.flush_buffer()?;
        }
        if !bytes[idx..].is_empty() {
            self.buffer[self.buffer_index..self.buffer_index + bytes[idx..].len()]
                .copy_from_slice(&bytes[idx..]);
            self.buffer_index += bytes[idx..].len();
        }
        Ok(())
    }

    pub fn flush_buffer(&mut self) -> Result<(), embedded_sdmmc::Error<SdCardError>> {
        if self.buffer_index == 0 {
            // nothing to flush
            return Ok(());
        }

        if let Some(log_file) = &mut self.log_file {
            if let Err(e) = log_file.write(&self.buffer[..self.buffer_index]) {
                error!("Error writing to log file: {:?}", e);
                return Err(e);
            }
        }
        self.buffer_index = 0;

        Ok(())
    }
}
/// Implement the `core::fmt::Write` trait for the logger
impl<SPI, CS, DELAY, const BUFSIZE: usize> core::fmt::Write for Logger<SPI, CS, DELAY, BUFSIZE>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    CS: OutputPin,
    DELAY: DelayUs<u8>,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_to_buffer(s).map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}
