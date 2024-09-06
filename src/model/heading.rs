#![allow(non_camel_case_types)]

use crate::system::millis::millis;
use core::marker::PhantomData;

use defmt::{error, info};
use embedded_hal::{delay::DelayNs, i2c::I2c};
use micromath::vector::Vector3d;
use mpu6050::Mpu6050;

// My MCU6050 calibration values
// ..	XAccel			                YAccel				        ZAccel			                XGyro			        YGyro			        ZGyro
//      [-2281,-2280] --> [-3,14]	    [1725,1726] --> [-15,4]	    [1581,1581] --> [16381,16389]	[97,98] --> [-2,1]	    [43,44] --> [-1,1]	    [20,20] --> [0,1]
//      [-2241,-2240] --> [-1,15]	    [1803,1804] --> [-10,6]	    [1573,1574] --> [16371,16394]	[98,99] --> [-2,1]	    [44,45] --> [0,5]	    [19,20] --> [0,4]
//      [-2239,-2238] --> [-3,16]	    [1804,1804] --> [0,5]	    [1575,1576] --> [16378,16395]	[98,99] --> [-1,2]	    [43,44] --> [-2,1]	    [19,20] --> [-1,2]

const HEADING_UPDATE_INTERVAL_MS: u32 = 1;

pub struct HeadingCalculator<TWI, DELAY> {
    heading: f32,
    gyro: Mpu6050<TWI>,
    last_update_rate: f32,
    last_update_millis: u32,
    inited: bool,
    _delay: PhantomData<DELAY>,
}

#[allow(dead_code, non_camel_case_types)]
impl<TWI, DELAY> HeadingCalculator<TWI, DELAY>
where
    TWI: I2c,
    DELAY: DelayNs,
{
    pub fn new(i2c: TWI, delay: &mut DELAY) -> Self {
        let mut gyro = Mpu6050::new(i2c);
        let mut inited = false;
        match gyro.init(delay) {
            Ok(_) => {
                info!("Gyro initialized");
                inited = true;
            }
            Err(_e) => {
                error!("Error initializing gyro");
            }
        };
        if let Err(_error) = gyro.set_gyro_range(mpu6050::device::GyroRange::D250) {
            error!("Error setting gyro range");
        }

        let cur_offsets = match gyro.get_gyro_offsets() {
            Ok(offsets) => offsets,
            Err(_error) => {
                error!("Error getting gyro offsets");
                Vector3d::<i32>::default()
            }
        };

        info!(
            "Got gyro offsets: x = {}, y = {}, z = {}",
            cur_offsets.x, cur_offsets.y, cur_offsets.z
        );

        Self {
            heading: 0.0,
            gyro,
            last_update_rate: 0.0,
            last_update_millis: millis(),
            inited,
            _delay: PhantomData,
        }
    }

    pub fn is_inited(&self) -> bool {
        self.inited
    }

    pub fn calibrate<F: FnMut(usize)>(&mut self, delay: &mut DELAY, callback: F) {
        if !self.is_inited() {
            error!("Gyro not initialized");
            return;
        }
        if let Err(_e) = self.gyro.calibrate_gyro(delay, callback) {
            error!("Error calibrating gyro");
        }
    }

    pub fn reset(&mut self) {
        self.heading = 0.0;
        self.last_update_rate = 0.0;
        self.last_update_millis = millis();
    }

    pub fn update(&mut self) -> f32 {
        if !self.is_inited() {
            return 0.0;
        }
        let now = millis();
        let delta_time = now - self.last_update_millis;
        if delta_time >= HEADING_UPDATE_INTERVAL_MS {
            match self.gyro.get_gyro_deg() {
                Ok(gyro) => {
                    self.last_update_millis = now;
                    // the heding is about the sensor's Z-axis
                    let delta_degs = gyro.z * (delta_time as f32 / 1000.0);
                    self.heading += delta_degs;
                    self.last_update_rate = gyro.z;

                    if self.heading > 180.0 {
                        self.heading -= 360.0;
                    } else if self.heading < -180.0 {
                        self.heading += 360.0;
                    }
                }
                Err(_error) => {
                    error!("Error reading gyro");
                }
            }
        }

        self.heading
    }

    pub fn heading(&mut self) -> f32 {
        self.update()
    }
}
