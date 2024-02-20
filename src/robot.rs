#![allow(non_camel_case_types, dead_code)]
mod debouncer;
pub mod file_storage;
mod motor_controller;
mod telemetry;

use crate::{
    model::{heading::HeadingCalculator, pid_controller::PIDController},
    robot::{
        debouncer::DebouncedButton, file_storage::FileStorage, motor_controller::MotorController,
        telemetry::straight_movement::StraightTelemetryRow,
    },
    system::{data::DataTable, millis::millis},
};
use adafruit_lcd_backpack::{LcdBackpack, LcdDisplayType};
use alloc::{
    format,
    string::{String, ToString},
    vec,
};
use core::{
    self,
    cell::{Cell, RefCell},
    fmt::Write,
};
use cortex_m::interrupt::Mutex;
use defmt::{debug, error, info, warn, Format};
use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    blocking::i2c::{Write as I2cWrite, WriteRead},
    digital::v2::{InputPin, OutputPin},
    PwmPin,
};
use micromath::F32Ext;
use rp_pico::hal::gpio;
use rp_pico::hal::{
    gpio::{
        bank0::{Gpio20, Gpio21},
        FunctionSio, Interrupt, SioInput,
    },
    pac::{self, interrupt},
};
use shared_bus::BusManagerCortexM;

const UP_ARROW_CHARACTER_DEFINITION: [u8; 8] = [
    0b00000, 0b00100, 0b01110, 0b10101, 0b00100, 0b00100, 0b00100, 0b00000,
];
const DOWN_ARROW_CHARACTER_DEFINITION: [u8; 8] = [
    0b00000, 0b00100, 0b00100, 0b00100, 0b10101, 0b01110, 0b00100, 0b00000,
];
const DEGREES_CHARACTER_DEFINITION: [u8; 8] = [
    0b00110, 0b01001, 0b01001, 0b00110, 0b00000, 0b00000, 0b00000, 0b00000,
];

const UP_ARROW_STRING: &str = "\x01";
const DOWN_ARROW_STRING: &str = "\x02";
const LEFT_ARROW_STRING: &str = "\x7F";
const RIGHT_ARROW_STRING: &str = "\x7E";
const DEGREES_STRING: &str = "\x03";

const DISPLAY_RESET_DELAY_MS: u32 = 5000;
//==============================================================================
// Directory and File namses
//
const LOG_DIR_NAME: &str = "log";
const LOG_INDEX_FILE_NAME: &str = "index.txt";

//==============================================================================
// Interrupt pins and counters for wheel encoders
//
type LeftWheelCounterPin = gpio::Pin<Gpio21, FunctionSio<SioInput>, gpio::PullUp>;
type RightWheelCounterPin = gpio::Pin<Gpio20, FunctionSio<SioInput>, gpio::PullUp>;

static LEFT_WHEEL_COUNTER_PIN: Mutex<RefCell<Option<LeftWheelCounterPin>>> =
    Mutex::new(RefCell::new(None));
static RIGHT_WHEEL_COUNTER_PIN: Mutex<RefCell<Option<RightWheelCounterPin>>> =
    Mutex::new(RefCell::new(None));

static LEFT_WHEEL_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
static RIGHT_WHEEL_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
//==============================================================================

//==============================================================================
// Constants for the robot
//
const WHEEL_DIAMETER: f32 = 67.0; // mm
const WHEEL_CIRCUMFERENCE: f32 = WHEEL_DIAMETER * core::f32::consts::PI;
const MOTOR_REDUCTION_RATIO: f32 = 46.8;
const ENCODER_TICKS_PER_REVOLUTION: f32 = 12.0;
const WHEEL_TICKS_PER_REVOLUTION: f32 = ENCODER_TICKS_PER_REVOLUTION * MOTOR_REDUCTION_RATIO;
const WHEEL_TICKS_PER_MM: f32 = WHEEL_TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;
const MM_PER_WHEEL_TICK: f32 = WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_REVOLUTION;

const BUTTON_DEBOUNCE_TIME_MS: u32 = 10;
const CONTROLLER_SAMPLE_PERIOD_MS: u32 = 50;

pub struct Robot<
    'a,
    INA1: OutputPin,
    INA2: OutputPin,
    INB1: OutputPin,
    INB2: OutputPin,
    ENA: PwmPin<Duty = u16>,
    ENB: PwmPin<Duty = u16>,
    BUTT1: InputPin,
    BUTT2: InputPin,
    TWI,
    TWI_ERR,
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    CS: OutputPin,
    DELAY,
> where
    TWI: I2cWrite<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8> + DelayUs<u8>,
{
    motors: MotorController<INA1, INA2, INB1, INB2, ENA, ENB>,
    button1: DebouncedButton<BUTT1, false, BUTTON_DEBOUNCE_TIME_MS>,
    button2: DebouncedButton<BUTT2, false, BUTTON_DEBOUNCE_TIME_MS>,
    pub heading_calculator: HeadingCalculator<shared_bus::I2cProxy<'a, Mutex<RefCell<TWI>>>, DELAY>,
    lcd: LcdBackpack<shared_bus::I2cProxy<'a, Mutex<RefCell<TWI>>>, DELAY>,
    sd_card: FileStorage<SPI, CS, DELAY>,
    reset_display_start_millis: u32,
    log_index: u32,
}

#[allow(dead_code, non_camel_case_types)]
impl<
        'a,
        INA1: OutputPin,
        INA2: OutputPin,
        INB1: OutputPin,
        INB2: OutputPin,
        ENA: PwmPin<Duty = u16>,
        ENB: PwmPin<Duty = u16>,
        BUTT1: InputPin,
        BUTT2: InputPin,
        TWI,
        TWI_ERR,
        SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
        CS: OutputPin,
        DELAY,
    > Robot<'a, INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, TWI_ERR, SPI, CS, DELAY>
where
    TWI: I2cWrite<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8> + DelayUs<u8> + Copy,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        ina1_pin: INA1,
        ina2_pin: INA2,
        inb1_pin: INB1,
        inb2_pin: INB2,
        ena_pin: ENA,
        enb_pin: ENB,
        button1_pin: BUTT1,
        button2_pin: BUTT2,
        i2c_manager: &'a BusManagerCortexM<TWI>,
        left_counter_pin: LeftWheelCounterPin,
        right_counter_pin: RightWheelCounterPin,
        mut sd_card: FileStorage<SPI, CS, DELAY>,
        delay: &mut DELAY,
    ) -> Self {
        // create the motor controller
        let motors = MotorController::new(ina1_pin, ina2_pin, inb1_pin, inb2_pin, ena_pin, enb_pin);

        // enable interrupts for wheel encoders
        cortex_m::interrupt::free(|cs| {
            left_counter_pin.set_interrupt_enabled(Interrupt::EdgeLow, true);
            LEFT_WHEEL_COUNTER_PIN
                .borrow(cs)
                .replace(Some(left_counter_pin));
            right_counter_pin.set_interrupt_enabled(Interrupt::EdgeLow, true);
            RIGHT_WHEEL_COUNTER_PIN
                .borrow(cs)
                .replace(Some(right_counter_pin));
        });
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        }

        let heading_i2c = i2c_manager.acquire_i2c();
        let mut heading_calculator = HeadingCalculator::new(heading_i2c, delay);
        heading_calculator.reset();

        let mut lcd = LcdBackpack::new(LcdDisplayType::Lcd16x2, i2c_manager.acquire_i2c(), *delay);
        match lcd.init() {
            Ok(_) => {
                info!("LCD initialized");
            }
            Err(e) => {
                error!("Error initializing LCD: {}", e);
            }
        };

        if let Err(error) =
            lcd.create_char(UP_ARROW_STRING.as_bytes()[0], UP_ARROW_CHARACTER_DEFINITION)
        {
            error!("Error creating up arrow character: {}", error);
        };
        if let Err(error) = lcd.create_char(
            DOWN_ARROW_STRING.as_bytes()[0],
            DOWN_ARROW_CHARACTER_DEFINITION,
        ) {
            error!("Error creating down arrow character: {}", error);
        };
        if let Err(error) =
            lcd.create_char(DEGREES_STRING.as_bytes()[0], DEGREES_CHARACTER_DEFINITION)
        {
            error!("Error creating degrees character: {}", error);
        };

        if let Err(error) = lcd
            .home()
            .and_then(LcdBackpack::clear)
            .and_then(|lcd| LcdBackpack::print(lcd, "Robot Started"))
            .and_then(|lcd| {
                write!(
                    lcd.set_cursor(0, 1)?,
                    "SD: {} GB",
                    sd_card.volume_size().unwrap_or(0) / 1_073_741_824
                )
                .map_err(|_e| adafruit_lcd_backpack::Error::FormattingError)
            })
        {
            error!("Error writing to LCD: {}", error);
        }

        // return the robot
        info!("Robot initialized");
        Self {
            motors,
            button1: DebouncedButton::<BUTT1, false, BUTTON_DEBOUNCE_TIME_MS>::new(button1_pin),
            button2: DebouncedButton::<BUTT2, false, BUTTON_DEBOUNCE_TIME_MS>::new(button2_pin),
            heading_calculator,
            lcd,
            sd_card,
            reset_display_start_millis: millis(), // start the display reset timer to clear the "started" message
            log_index: 0,
        }
    }

    /// Inits the Robot software. This function should be called after the robot is created.
    pub fn init(&mut self) -> Result<(), embedded_sdmmc::Error<embedded_sdmmc::SdCardError>> {
        self.init_log_file()?;

        Ok(())
    }

    /// Initializes the log file and log file index
    fn init_log_file(&mut self) -> Result<(), embedded_sdmmc::Error<embedded_sdmmc::SdCardError>> {
        // open log directory
        let root_dir = match self.sd_card.root_dir() {
            Some(dir) => dir,
            None => {
                error!("Error opening root dir");
                return Err(embedded_sdmmc::Error::DeviceError(
                    embedded_sdmmc::SdCardError::CardNotFound,
                ));
            }
        };
        let log_dir = match self.sd_card.open_dir(root_dir, LOG_DIR_NAME) {
            Some(dir) => dir,
            None => {
                error!("Error opening log dir");
                return Err(embedded_sdmmc::Error::DirAlreadyOpen);
            }
        };

        // open log index file
        let log_index = match self.sd_card.open_file_in_dir(
            LOG_INDEX_FILE_NAME,
            log_dir,
            embedded_sdmmc::Mode::ReadOnly,
        ) {
            Ok(mut file) => {
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
            Err(_e) => {
                warn!("Error opening log index file contents. Starting log index at 0");
                0
            }
        };

        // write the new log index back to the file
        match self.sd_card.open_file_in_dir(
            LOG_INDEX_FILE_NAME,
            log_dir,
            embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
        ) {
            Ok(mut file) => {
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
        self.log_index = log_index;

        Ok(())
    }

    pub fn test_sdcard(
        &mut self,
    ) -> Result<(), embedded_sdmmc::Error<embedded_sdmmc::SdCardError>> {
        let root_dir = self.sd_card.root_dir().unwrap();
        let mut found_files: vec::Vec<String> = vec::Vec::new();
        if let Err(e) = self.sd_card.iterate_dir(root_dir, |entry| {
            if entry.attributes.is_volume() {
                info!(
                    "Volume name is: {}, attributes = {:?}",
                    entry.name.to_string().as_str(),
                    entry.attributes,
                );
            } else if entry.attributes.is_directory() {
                info!(
                    "Root directory directory: {}, attributes = {:?}",
                    entry.name.to_string().as_str(),
                    entry.attributes,
                );
            } else {
                let filename = entry.name.to_string();
                info!(
                    "Root directory file: {}, attributes = {:?}",
                    filename.as_str(),
                    entry.attributes,
                );
                found_files.push(filename);
            }
        }) {
            error!("Error iterating root dir: {}", e);
        }

        for filename in found_files.iter() {
            if let Ok(mut file) = self.sd_card.open_file_in_dir(
                filename.as_str(),
                root_dir,
                embedded_sdmmc::Mode::ReadOnly,
            ) {
                let file_size = match file.length() {
                    Ok(size) => size,
                    Err(e) => {
                        error!("Error getting file size: {:?}", e);
                        0
                    }
                };

                let mut buffer = vec![0u8; file_size as usize];
                if let Err(e) = file.read(buffer.as_mut_slice()) {
                    error!("Error reading file: {:?}", e);
                } else {
                    let contents = String::from_utf8_lossy(&buffer).to_string();
                    info!(
                        "File '{}' contents:\n{:?}",
                        filename.as_str(),
                        contents.as_str()
                    );
                }
            }
        }
        Ok(())
    }
    /// This function is called in the main loop to allow the robot to handle state updates
    pub fn handle_loop(&mut self) {
        if self.reset_display_start_millis != 0
            && millis() - self.reset_display_start_millis > DISPLAY_RESET_DELAY_MS
        {
            debug!("Resetting LCD to idle message");
            if let Err(error) = core::write!(self.clear_lcd().set_lcd_cursor(0, 0), "Robot Idle") {
                error!("Error writing to LCD: {}", error.to_string().as_str());
            }
            self.reset_display_start_millis = 0;
        }
        // unset button press if button is not pressed
        self.button1.handle_loop();
        self.button2.handle_loop();

        self.heading_calculator.update();
    }

    pub fn display_text(&mut self, text: &str) {
        self.lcd.clear().ok();
        self.lcd.set_cursor(0, 0).ok();
        self.lcd.print(text).ok();
    }

    /// Resets the wheel counters to 0
    pub fn reset_wheel_counters(&mut self) {
        self.reset_left_wheel_counter();
        self.reset_right_wheel_counter();
    }

    /// Resets the left wheel counters to 0
    pub fn reset_left_wheel_counter(&mut self) {
        cortex_m::interrupt::free(|cs| {
            LEFT_WHEEL_COUNTER.borrow(cs).set(0);
        });
    }

    /// Resets the right wheel counters to 0
    pub fn reset_right_wheel_counter(&mut self) {
        cortex_m::interrupt::free(|cs| {
            RIGHT_WHEEL_COUNTER.borrow(cs).set(0);
        });
    }

    /// Returns the number of wheel ticks on the left wheel since the last reset
    pub fn get_left_wheel_counter(&self) -> u32 {
        cortex_m::interrupt::free(|cs| LEFT_WHEEL_COUNTER.borrow(cs).get())
    }

    /// Returns the number of wheel ticks on the right wheel since the last reset
    pub fn get_right_wheel_counter(&self) -> u32 {
        cortex_m::interrupt::free(|cs| RIGHT_WHEEL_COUNTER.borrow(cs).get())
    }

    /// Returns a duty value normalized to the max duty of the motor.
    /// The duty is clamped to the range [0, 1].
    fn noramlize_duty(&self, duty: f32) -> u16 {
        (duty.max(0.0).min(1.0) * self.motors.enable_pin_a().get_max_duty() as f32) as u16
    }

    /// returns true if the button 1 is newly pressed
    pub fn button1_pressed(&mut self) -> bool {
        self.button1.is_newly_pressed()
    }

    /// returns true if the button 2 is newly pressed
    pub fn button2_pressed(&mut self) -> bool {
        self.button2.is_newly_pressed()
    }

    /// Starts the robot display reset timer
    pub fn start_display_reset_timer(&mut self) {
        self.reset_display_start_millis = millis();
    }

    //--------------------------------------------------------------------------
    // Robot movement functions
    //--------------------------------------------------------------------------

    pub fn forward(&mut self, duty: f32) -> &mut Self {
        let normalized_duty = self.noramlize_duty(duty);
        self.motors.set_duty(normalized_duty, normalized_duty);
        self.motors.forward();
        self
    }

    pub fn straight(&mut self, distance_mm: u32, forward: bool) -> &mut Self {
        let direction_arrow = if forward {
            UP_ARROW_STRING
        } else {
            DOWN_ARROW_STRING
        };

        if let Err(error) = core::write!(
            self.clear_lcd().set_lcd_cursor(0, 0),
            "{} 0 / {} mm",
            direction_arrow,
            distance_mm,
        ) {
            error!("Error writing to LCD: {}", error.to_string().as_str());
        }

        let expected_ticks = (distance_mm as f32 * WHEEL_TICKS_PER_MM) as u32;
        info!(
            "Robot move straight, distance = {}, target ticks = {}",
            distance_mm, expected_ticks
        );
        self.reset_wheel_counters();

        let mut data_table = DataTable::<
            StraightTelemetryRow,
            { StraightTelemetryRow::DATA_COLUMNS },
        >::new(*StraightTelemetryRow::header());

        let mut traveled_ticks: u32 = 0;
        let mut last_update_millis = 0;
        let mut left_wheel_ticks = 0;
        let mut right_wheel_ticks = 0;

        let mut controller = PIDController::new(0.5, 0., 0.);
        // we want to go straight, so the setpoint is 0
        controller.set_setpoint(0.);
        self.heading_calculator.reset();

        self.motors
            .set_duty(self.noramlize_duty(1.), self.noramlize_duty(1.));
        data_table.append(StraightTelemetryRow::new(
            millis(),
            0,
            0,
            1.0,
            1.0,
            self.heading_calculator.heading(),
            0.,
        ));
        let mut update_count: u32 = 0;
        if forward {
            self.motors.forward();
        } else {
            self.motors.reverse();
        }

        while traveled_ticks < expected_ticks {
            cortex_m::interrupt::free(|cs| {
                left_wheel_ticks = LEFT_WHEEL_COUNTER.borrow(cs).get();
                right_wheel_ticks = RIGHT_WHEEL_COUNTER.borrow(cs).get();
            });
            traveled_ticks = (left_wheel_ticks + right_wheel_ticks) / 2;
            if millis() - last_update_millis > CONTROLLER_SAMPLE_PERIOD_MS {
                last_update_millis = millis();
                update_count += 1;

                let heading = self.heading_calculator.heading();
                let control_signal = controller.update(heading, last_update_millis);

                let mut cs_indicator: &str = direction_arrow;
                // positive control signal means turn left, a negative control signal means turn right
                let mut left_power: f32 = 1.;
                let mut right_power: f32 = 1.;
                if (forward && control_signal > 0.) || (!forward && control_signal < 0.) {
                    left_power = 1. - control_signal;
                    right_power = 1.0;
                    cs_indicator = LEFT_ARROW_STRING;
                } else if (forward && control_signal < 0.) || (!forward && control_signal > 0.) {
                    left_power = 1.0;
                    right_power = 1. + control_signal;
                    cs_indicator = RIGHT_ARROW_STRING;
                }
                self.motors.set_duty(
                    self.noramlize_duty(left_power),
                    self.noramlize_duty(right_power),
                );
                if update_count % 5 == 0 {
                    let _ = data_table.append(StraightTelemetryRow::new(
                        last_update_millis,
                        left_wheel_ticks,
                        right_wheel_ticks,
                        left_power,
                        right_power,
                        heading,
                        control_signal,
                    ));
                }

                if let Err(error) = core::write!(
                    self.set_lcd_cursor(0, 0),
                    "{} {} / {} mm",
                    direction_arrow,
                    (traveled_ticks as f32 * MM_PER_WHEEL_TICK) as i32,
                    distance_mm,
                ) {
                    error!("Error writing to LCD: {}", error.to_string().as_str());
                }
                if let Err(error) = core::write!(
                    self.set_lcd_cursor(0, 1),
                    "{} : cs={:.3}",
                    cs_indicator,
                    control_signal,
                ) {
                    error!("Error writing to LCD: {}", error.to_string().as_str());
                }
            }
            self.handle_loop();
        }
        if let Err(error) = core::write!(
            self.set_lcd_cursor(0, 1),
            "{} {}{} {}/ {}",
            left_wheel_ticks,
            LEFT_ARROW_STRING,
            RIGHT_ARROW_STRING,
            right_wheel_ticks,
            expected_ticks,
        ) {
            error!("Error writing to LCD: {}", error.to_string().as_str());
        }
        self.motors.stop();
        info!("Done with straight movement");
        data_table.append(StraightTelemetryRow::new(
            millis(),
            left_wheel_ticks,
            right_wheel_ticks,
            0.,
            0.,
            self.heading_calculator.heading(),
            0.,
        ));
        debug!("Completed data table");
        info!("Movement data = {}", data_table);
        self.start_display_reset_timer();

        self
    }

    pub fn min_turn_angle(&self) -> f32 {
        1.0
    }

    pub fn turn(&mut self, angle_degrees: i32) -> &mut Self {
        const TURN_MIN_POWER: f32 = 0.35;
        if angle_degrees.abs() < self.min_turn_angle() as i32 {
            debug!("Turn angle {} too small, not turning", angle_degrees);
            return self;
        }

        info!("Robot turn, angle = {}", angle_degrees);
        self.reset_wheel_counters();
        let direction_str = if angle_degrees > 0 {
            LEFT_ARROW_STRING
        } else {
            RIGHT_ARROW_STRING
        };
        if let Err(error) = core::write!(
            self.clear_lcd().set_lcd_cursor(0, 0),
            "{} 0 / {}\x03",
            direction_str,
            angle_degrees,
        ) {
            error!("Error writing to LCD: {}", error.to_string().as_str());
        }
        self.heading_calculator.reset();
        let mut current_angle = self.heading_calculator.heading();
        let mut last_adjust_angle = current_angle;

        // start the motors per right hand rule (postive angle = left turn, negative angle = right turn)
        // motor A is the left motor, motor B is the right motor
        self.motors
            .set_duty(self.noramlize_duty(1.), self.noramlize_duty(1.));
        if angle_degrees > 0 {
            self.motors.reverse_a();
            self.motors.forward_b();
        } else {
            self.motors.forward_a();
            self.motors.reverse_b();
        }

        while current_angle.abs() < (angle_degrees.abs() - 12) as f32 {
            current_angle = self.heading_calculator.heading();

            if (current_angle - last_adjust_angle).abs() > 5.0 {
                last_adjust_angle = current_angle;
                let abs_angle = angle_degrees.abs() as f32;
                let motor_power = TURN_MIN_POWER
                    + (1.0 - TURN_MIN_POWER) * ((abs_angle - current_angle.abs()) / abs_angle);
                self.motors.set_duty(
                    self.noramlize_duty(motor_power),
                    self.noramlize_duty(motor_power),
                );
            }
            if let Err(error) = core::write!(
                self.clear_lcd().set_lcd_cursor(0, 0),
                "{} {} / {}\x03",
                direction_str,
                current_angle as i32,
                angle_degrees,
            ) {
                error!("Error writing to LCD: {}", error.to_string().as_str());
            }
            self.handle_loop();
        }
        self.motors.stop();
        let start_millis = millis();
        let mut update_millis = start_millis;
        while millis() - start_millis < 1000 {
            self.handle_loop();
            if millis() - update_millis > 400 {
                update_millis = millis();
                current_angle = self.heading_calculator.heading();
                if let Err(error) = core::write!(
                    self.clear_lcd().set_lcd_cursor(0, 0),
                    "{} {} / {}\x03",
                    direction_str,
                    current_angle as i32,
                    angle_degrees,
                ) {
                    error!("Error writing to LCD: {}", error.to_string().as_str());
                }
            }
        }
        current_angle = self.heading_calculator.heading();
        if let Err(error) = core::write!(
            self.clear_lcd().set_lcd_cursor(0, 0),
            "{} {} / {}\x03",
            direction_str,
            current_angle as i32,
            angle_degrees,
        ) {
            error!("Error writing to LCD: {}", error.to_string().as_str());
        }
        info!("Done with turn movement");
        self.start_display_reset_timer();
        self
    }

    pub fn stop(&mut self) -> &mut Self {
        self.motors.set_duty(0, 0);
        self
    }

    //--------------------------------------------------------------------------
    // LCD functions
    //--------------------------------------------------------------------------

    /// clears the robot's LCD Screen
    pub fn clear_lcd(&mut self) -> &mut Self {
        self.lcd.clear().ok();
        self
    }

    /// set the cursor on the robot's LCD cursor
    pub fn set_lcd_cursor(&mut self, col: u8, row: u8) -> &mut Self {
        self.lcd.set_cursor(col, row).ok();
        self
    }
}

/// Implements the defmt::Format trait for the Robot struct, allowing the Robot object to be printed with defmt
impl<
        'a,
        INA1: OutputPin,
        INA2: OutputPin,
        INB1: OutputPin,
        INB2: OutputPin,
        ENA: PwmPin<Duty = u16>,
        ENB: PwmPin<Duty = u16>,
        BUTT1: InputPin,
        BUTT2: InputPin,
        TWI,
        TWI_ERR,
        SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
        CS: OutputPin,
        DELAY,
    > Format
    for Robot<'a, INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, TWI_ERR, SPI, CS, DELAY>
where
    TWI: I2cWrite<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8> + DelayUs<u8> + Copy,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
{
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Robot< left = {}, right = {} >",
            self.get_left_wheel_counter(),
            self.get_right_wheel_counter()
        );
    }
}

/// Implement the `core::fmt::Write` trait for the robot, allowing us to use the `core::write!` macro
impl<
        'a,
        INA1: OutputPin,
        INA2: OutputPin,
        INB1: OutputPin,
        INB2: OutputPin,
        ENA: PwmPin<Duty = u16>,
        ENB: PwmPin<Duty = u16>,
        BUTT1: InputPin,
        BUTT2: InputPin,
        TWI,
        TWI_ERR,
        SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
        CS: OutputPin,
        DELAY,
    > core::fmt::Write
    for Robot<'a, INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, TWI_ERR, SPI, CS, DELAY>
where
    TWI: I2cWrite<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8> + DelayUs<u8> + Copy,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.lcd.write_str(s)
    }
}

/// Interrupt handler for the wheel encoders
#[interrupt]
fn IO_IRQ_BANK0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(pin) = LEFT_WHEEL_COUNTER_PIN.borrow(cs).borrow_mut().as_mut() {
            if pin.interrupt_status(Interrupt::EdgeLow) {
                LEFT_WHEEL_COUNTER
                    .borrow(cs)
                    .set(LEFT_WHEEL_COUNTER.borrow(cs).get() + 1);
                pin.clear_interrupt(Interrupt::EdgeLow);
            }
        }
    });
    cortex_m::interrupt::free(|cs| {
        if let Some(pin) = RIGHT_WHEEL_COUNTER_PIN.borrow(cs).borrow_mut().as_mut() {
            if pin.interrupt_status(Interrupt::EdgeLow) {
                RIGHT_WHEEL_COUNTER
                    .borrow(cs)
                    .set(RIGHT_WHEEL_COUNTER.borrow(cs).get() + 1);
                pin.clear_interrupt(Interrupt::EdgeLow);
            }
        }
    });
}
