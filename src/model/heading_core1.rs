use bsp::hal::{
    fugit::{HertzU32, Rate},
    gpio::{
        bank0::{Gpio4, Gpio5},
        FunctionI2C, Pin, PullUp,
    },
    multicore::Stack,
    pac,
    sio::Sio,
    Timer,
};
use core::cell::{Cell, RefCell};
use critical_section::Mutex;
use embedded_hal::i2c::{self, I2c};
use mpu6050::Mpu6050;
use rp_pico as bsp;

const HEADING_UPDATE_MICROS: u32 = 2 * 1000; // 2ms

pub enum HeadingTaskCommands {
    Shutdown,
    Status,
    SetOffsets,
    Unknown,
}

impl Into<u32> for HeadingTaskCommands {
    fn into(self) -> u32 {
        match self {
            HeadingTaskCommands::Shutdown => 0xFFFFFFFF,
            HeadingTaskCommands::Status => 1,
            HeadingTaskCommands::SetOffsets => 2,
            HeadingTaskCommands::Unknown => 0,
        }
    }
}

impl From<u32> for HeadingTaskCommands {
    fn from(val: u32) -> Self {
        match val {
            0xFFFFFFFF => HeadingTaskCommands::Shutdown,
            1 => HeadingTaskCommands::Status,
            2 => HeadingTaskCommands::SetOffsets,
            _ => HeadingTaskCommands::Unknown,
        }
    }
}

pub enum HeadingTaskStatus {
    CalculatingHeading,
    Calibrating,
    Done,
    Unknown,
}

impl Into<u32> for HeadingTaskStatus {
    fn into(self) -> u32 {
        match self {
            HeadingTaskStatus::CalculatingHeading => 1,
            HeadingTaskStatus::Calibrating => 2,
            HeadingTaskStatus::Done => 0xFFFFFFFF,
            HeadingTaskStatus::Unknown => 0,
        }
    }
}

impl From<u32> for HeadingTaskStatus {
    fn from(val: u32) -> Self {
        match val {
            1 => HeadingTaskStatus::CalculatingHeading,
            2 => HeadingTaskStatus::Calibrating,
            0xFFFFFFFF => HeadingTaskStatus::Done,
            _ => HeadingTaskStatus::Unknown,
        }
    }
}

/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static
/// values is reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything separately
/// and modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte
/// alignment, which allows the stack guard to take up the least amount of
/// usable RAM.
static mut CORE1_STACK: Stack<4096> = Stack::new();

// The basic design for dedicating core1 to heading calculation
//
// There are two tasks that core1 will be responsible for:
// 1. Reading the gyro sensor and calculating the heading
// 2. Calibrating the gyro on demand
//
// The issue is each are seperate tasks, so we will need to communication from core0 to core1
// when it needs to stop the heading calculation and start the calibration. This will be done
// by sending a sentinal value from core0 to core1 using the SIO FIFO. The sentinal value for
// shutting down a task will be `HeadingTaskCommands::Shutdown`. When a task receives this value, it
// will close up any critical sections and enter into a busy wait loop it only listens to
// the SIO FIFO for a status request, `HeadingTaskCommands::Status`. When it receives this value
// it will send back the current status of the task, which will be `HeadingTaskCommands::Shutdown`.
// To start a new task, whether it be the same task or a different one, core0 will simply
// spawn the task again, which shuts down the current task and starts the new one.
//
// ** Calculating the heading task **
// The task for calculating heading will get the updated gyro readings from the sensor at
// a fixed interval, calculate the heading, and the setting it to a global variable that
// is wrapped in a Mutex. A critical section will be used to access the global variable.
// While running the task will monitor the SIO FIFO for either a shutdown or status request.
// If it receives a shutdown, it will close up any critical sections and enter into a busy
// wait loop. If it receives a status request, it will send back the current status of the
// task, which will be either TASK_HEADING_CALCULATING = 0xFFFFFFFD or TASK_DONE = 0xFFFFFFFF.
//
// ** Calibrating the gyro task **

static HEADING_VALUE: Mutex<Cell<HeadingData>> = Mutex::new(Cell::new(HeadingData {
    heading: 0.0,
    last_update_rate: 0.0,
    last_update_ticks: 0,
}));

#[derive(Debug, Clone, Copy)]
struct HeadingData {
    heading: f64,
    last_update_rate: f64,
    last_update_ticks: u64,
}
pub struct HeadingManager<'a> {
    core1: &'a mut rp_pico::hal::multicore::Core<'a>,
    sys_freq: Rate<u32, 1, 1>,
}

impl<'a> HeadingManager<'a> {
    pub fn new(
        core1: &'a mut rp_pico::hal::multicore::Core<'a>,
        sys_freq: Rate<u32, 1, 1>,
    ) -> Self {
        Self {
            core1: core1,
            sys_freq: sys_freq,
        }
    }

    pub fn start_heading_calculation(
        &mut self,
        timer: Timer,
        i2c0_sda_pin: Pin<Gpio4, FunctionI2C, PullUp>,
        i2c0_scl_pin: Pin<Gpio5, FunctionI2C, PullUp>,
    ) {
        // spawn the heading calculation task on core1
        let sys_freq = self.sys_freq; // value that can be moved into the closure
        let mut timer = timer.clone();
        let res = self.core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            // do the initialization in a critical section
            // set up environment
            let mut pac = unsafe { pac::Peripherals::steal() };
            let core = unsafe { pac::CorePeripherals::steal() };
            let mut sio = Sio::new(pac.SIO);

            let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq.to_Hz());

            //     let sda0_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio4.reconfigure();
            //     let scl0_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio5.reconfigure();
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
            // start the heading calculation task
            loop {
                // get the current gyro readings
                // calculate the heading
                // set the heading to the global variable
                // wait for the next update interval
                // check the SIO FIFO for a shutdown or status request
                delay.delay_us(HEADING_UPDATE_MICROS / 10);
                if let Some(cmd) = sio.fifo.read() {
                    match cmd.into() {
                        HeadingTaskCommands::Shutdown => {
                            // close up any critical sections
                            // enter into a busy wait loop
                            // only listen to the SIO FIFO for a status request
                            // send back the current status of the task
                            // which will be `HeadingTaskCommands::Shutdown`
                            break;
                        }
                        HeadingTaskCommands::Status => {
                            // send back the current status of the task
                            // which will be `HeadingTaskCommands::Shutdown`
                            sio.fifo.write(HeadingTaskStatus::CalculatingHeading.into());
                        }
                        _ => {}
                    }
                }
                HeadingManager::update_heading(&mut gyro, &mut timer);
            }
        });
    }
    /// runs on core 1
    fn update_heading<I2C>(gyro: &mut Mpu6050<I2C>, timer: &mut Timer)
    where
        I2C: embedded_hal::i2c::I2c,
    {
        let current_micros = timer.get_counter().ticks();
        let last_data = critical_section::with(|cs| HEADING_VALUE.borrow(cs).get());

        if current_micros - last_data.last_update_ticks < HEADING_UPDATE_MICROS as u64 {
            // too soon to update the heading
            return;
        }

        let gyro_reading = match gyro.get_gyro_deg() {
            Ok(reading) => reading,
            Err(_e) => {
                // error!("Error getting gyro readings");
                return;
            }
        };

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
}
