use crate::system::millis::millis;
use alloc::rc::Rc;
use core::cell::RefCell;
use cortex_m::delay::Delay;
use defmt::{error, info, trace};
use embedded_hal::blocking::i2c::{Write, WriteRead};
use mpu6050::Mpu6050;

pub struct HeadingCalculator<TWI> {
    heading: f32,
    gyro: Mpu6050<TWI>,
    last_update_rate: f32,
    last_update_millis: u32,
}

#[allow(dead_code, non_camel_case_types)]
impl<TWI, TWI_ERR> HeadingCalculator<TWI>
where
    TWI: Write<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
{
    pub fn new(i2c: &Rc<RefCell<TWI>>, delay: &mut Delay) -> Self {
        let mut gyro = Mpu6050::new(i2c.clone());
        match gyro.init(delay) {
            Ok(_) => {
                info!("Gyro initialized");
            }
            Err(e) => {
                error!("Error initializing gyro: {}", e);
            }
        };
        if let Err(_error) = gyro.set_gyro_range(mpu6050::device::GyroRange::D250) {
            error!("Error setting gyro range");
        }
        Self {
            heading: 0.0,
            gyro,
            last_update_rate: 0.0,
            last_update_millis: 0,
        }
    }

    pub fn reset(&mut self) {
        self.heading = 0.0;
        self.last_update_rate = 0.0;
        self.last_update_millis = millis();
    }

    pub fn update(&mut self) -> f32 {
        let now = millis();
        let delta_time = now - self.last_update_millis;
        if delta_time > 50 {
            trace!("update heading");
            if let Ok(gyro) = self.gyro.get_gyro() {
                // the heding is about the sensor's Z-axis
                let delta_rads = gyro.z * delta_time as f32 / 1000.0;
                self.heading += delta_rads;
                self.last_update_rate = gyro.z;
                trace!("delta_rads: {}, heading: {}", delta_rads, self.heading);
            }
        }

        self.heading
    }

    pub fn heading(&mut self) -> f32 {
        self.update()
    }
}
