use crate::system::millis::millis;
use alloc::rc::Rc;
use core::{borrow::BorrowMut, cell::RefCell, marker::PhantomData};
use defmt::{debug, error, info};
use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Write, WriteRead},
};
use mpu6050::Mpu6050;

pub struct HeadingCalculator<TWI, DELAY> {
    heading: f32,
    gyro: Mpu6050<TWI>,
    last_update_rate: f32,
    last_update_millis: u32,
    _delay: PhantomData<DELAY>,
}

#[allow(dead_code, non_camel_case_types)]
impl<TWI, TWI_ERR, DELAY> HeadingCalculator<TWI, DELAY>
where
    TWI: Write<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    DELAY: DelayMs<u8>,
{
    pub fn new(i2c: &Rc<RefCell<TWI>>, delay: &mut Rc<RefCell<DELAY>>) -> Self {
        let mut gyro = Mpu6050::new(i2c.clone());
        let delay_borrowed = delay.borrow_mut();
        let delay_ref = &mut *(**delay_borrowed).borrow_mut();
        match gyro.init(delay_ref) {
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
            _delay: PhantomData,
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
            if let Ok(gyro) = self.gyro.get_gyro() {
                // the heding is about the sensor's Z-axis
                let delta_degrees = gyro.z * (delta_time as f32 / 1000.0);
                self.heading += delta_degrees;
                self.last_update_rate = gyro.z;
                debug!(
                    "{}: delta_rads: {}, heading: {}",
                    self.gyro, delta_degrees, self.heading
                );
            }
        }

        self.heading
    }

    pub fn heading(&mut self) -> f32 {
        self.update()
    }
}
