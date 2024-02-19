use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::OutputPin;
// Link in the embedded_sdmmc crate.
// The `SdMmcSpi` is used for block level access to the card.
// And the `VolumeManager` gives access to the FAT filesystem functions.
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeManager};

// Get the file open mode enum:
use defmt::info;

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
    volume_mgr: VolumeManager<SdCard<SPI, CS, DELAY>, DummyTimesource>,
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
        let volume_mgr = VolumeManager::new(sd_card, DummyTimesource::default());

        Self { volume_mgr }
    }
}
