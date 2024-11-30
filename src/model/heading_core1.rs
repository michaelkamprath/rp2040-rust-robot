use bsp::hal::{
    fugit::{HertzU32, Rate},
    gpio::{
        bank0::{Gpio4, Gpio5},
        FunctionI2C, Pin, PullUp,
    },
    multicore::Stack,
    pac, Timer,
};
use core::cell::{Cell, RefCell};
use critical_section::Mutex;
use defmt::error;
use embedded_hal::digital::OutputPin;
use micromath::vector::Vector3d;
use mpu6050::Mpu6050;
use rp_pico as bsp;

const HEADING_UPDATE_MICROS: u32 = 5 * 1000; // 5ms

/// Stack for core 1
static mut CORE1_STACK: Stack<4096> = Stack::new();

static HEADING_VALUE: Mutex<Cell<HeadingData>> = Mutex::new(Cell::new(HeadingData {
    heading: 0.0,
    last_update_rate: 0.0,
    last_update_ticks: 0,
}));

static GYRO_OFFSETS: Mutex<Cell<Vector3d<i16>>> =
    Mutex::new(Cell::new(Vector3d::<i16> { x: 0, y: 0, z: 0 }));

#[derive(Debug, Clone, Copy)]
struct HeadingData {
    heading: f64,
    last_update_rate: f64,
    last_update_ticks: u64,
}

/// A core 0 object that manages the heading calculation on core 1.
pub struct HeadingManager<'a> {
    core1: &'a mut rp_pico::hal::multicore::Core<'a>,
    sys_freq: Rate<u32, 1, 1>,
    timer: Timer,
}

impl<'a> HeadingManager<'a> {
    pub fn new(
        core1: &'a mut rp_pico::hal::multicore::Core<'a>,
        sys_freq: Rate<u32, 1, 1>,
        timer: &mut Timer,
    ) -> Self {
        Self {
            core1,
            sys_freq,
            timer: *timer,
        }
    }

    /// Start the heading calculation task on core 1, initializing I2C bus 0 and the gyro sensor.
    pub fn start_heading_calculation<LEDPIN: OutputPin + Send + 'static>(
        &mut self,
        timer: Timer,
        i2c0_sda_pin: Pin<Gpio4, FunctionI2C, PullUp>,
        i2c0_scl_pin: Pin<Gpio5, FunctionI2C, PullUp>,
        mut led_pin: LEDPIN,
        gyro_offsets: Vector3d<i16>,
    ) {
        critical_section::with(|cs| {
            GYRO_OFFSETS.borrow(cs).set(gyro_offsets);
        });

        // spawn the heading calculation task on core1
        let sys_freq = self.sys_freq; // value that can be moved into the closure
        let mut timer = timer;
        #[allow(static_mut_refs)]
        let _res = self.core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            // LED is used to indicate that the gyro is good to go. make sure it is off to start
            if let Err(_e) = led_pin.set_low() {
                error!("Error setting LED pin low");
            };

            // do the initialization in a critical section
            // set up environment
            let mut pac = unsafe { pac::Peripherals::steal() };
            let core = unsafe { pac::CorePeripherals::steal() };

            let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq.to_Hz());
            // set up the I2C controller
            let i2c0 = bsp::hal::I2C::new_controller(
                pac.I2C0,
                i2c0_sda_pin,
                i2c0_scl_pin,
                HertzU32::from_raw(400_000),
                &mut pac.RESETS,
                sys_freq,
            );
            let i2c0_ref_cell = RefCell::new(i2c0);
            let i2c0_mutex = critical_section::Mutex::new(i2c0_ref_cell);

            // set up the gyro sensor   ]
            let mut gyro = Mpu6050::new(embedded_hal_bus::i2c::CriticalSectionDevice::new(
                &i2c0_mutex,
            ));
            if let Err(_e) = gyro.init(&mut timer) {
                panic!("Error initializing gyro");
            }

            // sett the offsets
            let gyro_offsets = critical_section::with(|cs| GYRO_OFFSETS.borrow(cs).get());
            // if the offsets are all zero, then we calibrate. otherwise, we set the offsets
            if gyro_offsets.x == 0 && gyro_offsets.y == 0 && gyro_offsets.z == 0 {
                defmt::info!("Configured gyro offsets are 0. Calibrating gyro ...");
                if let Err(_e) = gyro.calibrate_gyro(&mut timer, |count| {
                    defmt::info!("Calibrating gyro: {}", count);
                }) {
                    defmt::error!("Error calibrating gyro");
                }
                let new_offsets: Vector3d<i16> = match gyro.get_gyro_offsets() {
                    Ok(offsets) => Vector3d::<i16> {
                        x: offsets.x as i16,
                        y: offsets.y as i16,
                        z: offsets.z as i16,
                    },
                    Err(_e) => {
                        defmt::error!("Error getting gyro offsets");
                        Vector3d::<i16> { x: 0, y: 0, z: 0 }
                    }
                };
                critical_section::with(|cs| {
                    GYRO_OFFSETS.borrow(cs).set(new_offsets);
                });
            } else {
                defmt::debug!(
                    "Setting gyro offsets: {{x: {}, y: {}, z: {} }}",
                    gyro_offsets.x,
                    gyro_offsets.y,
                    gyro_offsets.z
                );
                if let Err(_e) =
                    gyro.set_gyro_offsets(gyro_offsets.x, gyro_offsets.y, gyro_offsets.z)
                {
                    defmt::error!("Error setting gyro offsets");
                }
            }
            defmt::info!("Gyro initialized : {}", gyro);
            if let Err(_e) = led_pin.set_high() {
                error!("Error setting LED pin high");
            };

            critical_section::with(|cs| {
                HEADING_VALUE.borrow(cs).set(HeadingData {
                    heading: 0.0,
                    last_update_rate: 0.0,
                    last_update_ticks: timer.get_counter().ticks(),
                });
            });

            // start the heading calculation task
            loop {
                delay.delay_us(HEADING_UPDATE_MICROS / 10);
                HeadingManager::update_heading(&mut gyro, &mut timer);
            }
        });
    }
    /// runs on core 1
    fn update_heading<I2C>(gyro: &mut Mpu6050<I2C>, timer: &mut Timer)
    where
        I2C: embedded_hal::i2c::I2c,
    {
        let last_data = critical_section::with(|cs| HEADING_VALUE.borrow(cs).get());
        if timer.get_counter().ticks() - last_data.last_update_ticks < HEADING_UPDATE_MICROS as u64
        {
            // too soon to update the heading
            return;
        }

        let gyro_reading = match gyro.get_gyro_deg() {
            Ok(reading) => reading,
            Err(_e) => {
                defmt::error!("Error getting gyro readings vis I2C");
                return;
            }
        };
        let current_micros = timer.get_counter().ticks();

        let delta_micros = current_micros - last_data.last_update_ticks;
        let delta_time_s = delta_micros as f64 / 1_000_000.0;

        let delta_heading =
            (last_data.last_update_rate + gyro_reading.z as f64) * delta_time_s / 2.0;
        let mut new_heading = last_data.heading + delta_heading;
        if new_heading > 180.0 {
            new_heading -= 360.0;
        } else if new_heading < -180.0 {
            new_heading += 360.0;
        }
        let new_rate = gyro_reading.z as f64;
        critical_section::with(|cs| {
            HEADING_VALUE.borrow(cs).set(HeadingData {
                heading: new_heading,
                last_update_rate: new_rate,
                last_update_ticks: current_micros,
            });
        });
    }

    /// runs on core 0
    pub fn get_heading(&self) -> f32 {
        // DEBUG get status
        // send status request to core1

        critical_section::with(|cs| HEADING_VALUE.borrow(cs).get().heading as f32)
    }

    /// reset the heading to 0
    pub fn reset_heading(&self) {
        critical_section::with(|cs| {
            HEADING_VALUE.borrow(cs).set(HeadingData {
                heading: 0.0,
                last_update_rate: 0.0,
                last_update_ticks: self.timer.get_counter().ticks(),
            });
        });
    }

    /// get the gyro offsets
    pub fn get_gyro_offsets(&self) -> Vector3d<i16> {
        critical_section::with(|cs| GYRO_OFFSETS.borrow(cs).get())
    }
}
