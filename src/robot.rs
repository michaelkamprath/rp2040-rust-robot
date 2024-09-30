#![allow(non_camel_case_types, dead_code)]
mod config;
mod debouncer;
pub mod file_storage;
mod motor_controller;
mod telemetry;

use crate::{
    model::{heading::HeadingCalculator, pid_controller::PIDController},
    robot::{
        config::Config,
        debouncer::DebouncedButton,
        file_storage::{logger::Logger, FileStorage},
        motor_controller::MotorController,
        telemetry::straight_movement::StraightTelemetryRow,
    },
    system::millis::millis,
};
use alloc::{
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
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    i2c::I2c,
    pwm::SetDutyCycle,
    spi::SpiDevice,
};
use embedded_hal_bus::i2c::CriticalSectionDevice;
use i2c_character_display::{AdafruitLCDBackpack, LcdDisplayType};
use micromath::F32Ext;
use rp_pico::hal::gpio;
use rp_pico::hal::{
    gpio::{
        bank0::{Gpio20, Gpio21},
        FunctionSio, Interrupt, SioInput,
    },
    pac::{self, interrupt},
};

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

const CONFIG_DIRECTORY: &str = "config";
const CONFIG_FILE: &str = "config.ini";

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

const BUTTON_DEBOUNCE_TIME_MS: u32 = 10;
const CONTROLLER_SAMPLE_PERIOD_MS: u32 = 25;

pub struct Robot<
    'a,
    INA1: OutputPin,
    INA2: OutputPin,
    INB1: OutputPin,
    INB2: OutputPin,
    ENA: SetDutyCycle,
    ENB: SetDutyCycle,
    BUTT1: InputPin,
    BUTT2: InputPin,
    TWI,
    SPI_DEV: embedded_hal::spi::SpiDevice<u8>,
    DELAY,
> where
    TWI: embedded_hal::i2c::I2c,
    DELAY: DelayNs,
{
    motors: MotorController<INA1, INA2, INB1, INB2, ENA, ENB>,
    button1: DebouncedButton<BUTT1, false, BUTTON_DEBOUNCE_TIME_MS>,
    button2: DebouncedButton<BUTT2, false, BUTTON_DEBOUNCE_TIME_MS>,
    pub heading_calculator:
        HeadingCalculator<embedded_hal_bus::i2c::CriticalSectionDevice<'a, TWI>, DELAY>,
    lcd: AdafruitLCDBackpack<CriticalSectionDevice<'a, TWI>, DELAY>,
    pub sd_card: FileStorage<SPI_DEV, DELAY>,
    reset_display_start_millis: u32,
    log_index: u32,
    pub logger: Logger<SPI_DEV, DELAY, 128>,
    idle_message_line2: Option<String>,
    config: Config,
}

#[allow(dead_code, non_camel_case_types)]
impl<
        'a,
        INA1: OutputPin,
        INA2: OutputPin,
        INB1: OutputPin,
        INB2: OutputPin,
        ENA: SetDutyCycle,
        ENB: SetDutyCycle,
        BUTT1: InputPin,
        BUTT2: InputPin,
        TWI,
        SPI_DEV: embedded_hal::spi::SpiDevice<u8>,
        DELAY,
    > Robot<'a, INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, SPI_DEV, DELAY>
where
    TWI: embedded_hal::i2c::I2c,
    DELAY: DelayNs + Clone,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        ina1_pin: INA1,
        ina2_pin: INA2,
        inb1_pin: INB1,
        inb2_pin: INB2,
        duty_a: ENA,
        duty_b: ENB,
        button1_pin: BUTT1,
        button2_pin: BUTT2,
        i2c_refcell: &'a critical_section::Mutex<RefCell<TWI>>,
        left_counter_pin: LeftWheelCounterPin,
        right_counter_pin: RightWheelCounterPin,
        mut sd_card: FileStorage<SPI_DEV, DELAY>,
        delay: &mut DELAY,
    ) -> Self {
        // create the motor controller
        let motors = MotorController::new(ina1_pin, ina2_pin, inb1_pin, inb2_pin, duty_a, duty_b);

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

        // let i2c_device = embedded_hal_bus::i2c::RefCellDevice::new(i2c_refcell);
        let i2c_device = embedded_hal_bus::i2c::CriticalSectionDevice::new(i2c_refcell);
        let mut lcd = AdafruitLCDBackpack::new(i2c_device, LcdDisplayType::Lcd16x2, delay.clone());
        match lcd.init() {
            Ok(_) => {
                info!("LCD initialized");
            }
            Err(_e) => {
                error!("Error initializing LCD");
            }
        };

        debug!("Creating custom character 1");
        if let Err(_e) =
            lcd.create_char(UP_ARROW_STRING.as_bytes()[0], UP_ARROW_CHARACTER_DEFINITION)
        {
            error!("Error creating up arrow character");
        };
        if let Err(_e) = lcd.create_char(
            DOWN_ARROW_STRING.as_bytes()[0],
            DOWN_ARROW_CHARACTER_DEFINITION,
        ) {
            error!("Error creating down arrow character");
        };
        debug!("Creating custom character 2");
        if let Err(_e) = lcd.create_char(DEGREES_STRING.as_bytes()[0], DEGREES_CHARACTER_DEFINITION)
        {
            error!("Error creating degrees character");
        };
        debug!("LCD characters created");

        debug!("Writing to LCD first message");
        if let Err(_e) = lcd
            .home()
            .and_then(AdafruitLCDBackpack::clear)
            .and_then(|lcd| AdafruitLCDBackpack::print(lcd, "Calibrating Gyro"))
        {
            error!("Error writing to LCD");
        }
        delay.delay_ms(5000);
        let mut heading_calculator = HeadingCalculator::new(
            embedded_hal_bus::i2c::CriticalSectionDevice::new(i2c_refcell),
            delay,
        );
        heading_calculator.reset();

        if let Err(_e) = lcd
            .home()
            .and_then(AdafruitLCDBackpack::clear)
            .and_then(|lcd| AdafruitLCDBackpack::print(lcd, "Robot Started"))
            .and_then(|lcd| {
                write!(
                    lcd.set_cursor(0, 1)?,
                    "SD: {} GB",
                    sd_card.volume_size().unwrap_or(0) / 1_073_741_824
                )
                .map_err(i2c_character_display::Error::FormattingError)
            })
        {
            error!("Error writing to LCD");
        }

        let logger = Logger::new(Logger::<SPI_DEV, DELAY, 128>::init_log_file(&mut sd_card));
        Self {
            motors,
            button1: DebouncedButton::<BUTT1, false, BUTTON_DEBOUNCE_TIME_MS>::new(button1_pin),
            button2: DebouncedButton::<BUTT2, false, BUTTON_DEBOUNCE_TIME_MS>::new(button2_pin),
            heading_calculator,
            lcd,
            sd_card,
            reset_display_start_millis: millis(), // start the display reset timer to clear the "started" message
            log_index: 0,
            logger,
            idle_message_line2: None,
            config: Config::new(),
        }
    }

    /// Inits the Robot software. This function should be called after the robot is created.
    pub fn init(&mut self) -> Result<(), embedded_sdmmc::Error<embedded_sdmmc::SdCardError>> {
        // load configuration from the SD card
        if self.sd_card.root_dir().is_some() {
            let root_dir = self.sd_card.root_dir().unwrap();
            let config_dir = match self.sd_card.open_directory(root_dir, CONFIG_DIRECTORY) {
                Some(d) => d,
                None => {
                    error!("Error opening config directory");
                    return Err(embedded_sdmmc::Error::NotFound);
                }
            };
            if let Ok(mut config_file) = self.sd_card.open_file_in_dir(
                CONFIG_FILE,
                config_dir,
                embedded_sdmmc::Mode::ReadOnly,
            ) {
                let config_length = config_file.length()?;
                let mut config_buffer = vec![0u8; config_length as usize];
                let bytes_read = config_file.read(&mut config_buffer)?;
                let config_str =
                    core::str::from_utf8(&config_buffer[0..bytes_read]).map_err(|e| {
                        error!("Error reading config file: {}", e.to_string().as_str());
                        embedded_sdmmc::Error::FormatError("Error reading config file")
                    })?;
                debug!("Config file contents:\n{}", config_str);
                self.config.set_config_values(config_str);
            } else {
                debug!("Config file not found, using defaults");
            }
        } else {
            warn!("Could not load configuration. Using defaults.");
        }
        writeln!(self.logger, "Robot started\n{}\n", self.config)
            .map_err(|_e| embedded_sdmmc::SdCardError::WriteError)?;
        info!("Robot initialized\n{}\n", self.config);
        Ok(())
    }

    pub fn calibrate_gyro(&mut self, delay: &mut DELAY) {
        self.heading_calculator.calibrate(delay, |step| {
            if let Ok(lcd) = self.lcd.set_cursor(0, 1) {
                write!(lcd, "iteration: {}", step).ok();
            }
        });
    }

    /// This function is called in the main loop to allow the robot to handle state updates
    pub fn handle_loop(&mut self) {
        if self.reset_display_start_millis != 0
            && millis() - self.reset_display_start_millis > DISPLAY_RESET_DELAY_MS
        {
            debug!("Resetting LCD to idle message");
            let idle_message = self.config.idle_message.clone();
            if let Err(error) =
                core::write!(self.clear_lcd().set_lcd_cursor(0, 0), "{}", idle_message)
            {
                error!("Error writing to LCD: {}", error.to_string().as_str());
            }
            if self.idle_message_line2.is_some() {
                let message = self.idle_message_line2.as_ref().unwrap().clone();
                if let Err(error) = core::write!(self.set_lcd_cursor(0, 1), "{}", message) {
                    error!("Error writing to LCD: {}", error.to_string().as_str());
                }
            }
            self.logger.flush_buffer().ok();
            self.reset_display_start_millis = 0;
        }
        // unset button press if button is not pressed
        self.button1.handle_loop();
        self.button2.handle_loop();

        self.heading_calculator.update();
    }

    pub fn set_idle_message_line2(&mut self, message: Option<String>) {
        self.idle_message_line2 = message;
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
        (duty.clamp(0.0, 1.0) * self.motors.enable_pin_a().get_max_duty() as f32) as u16
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

    /// Sets the robot display to the idle message
    pub fn set_display_to_idle(&mut self) {
        self.reset_display_start_millis = 1;
    }

    /// Clears the robot display reset timer
    pub fn clear_display_reset_timer(&mut self) {
        self.reset_display_start_millis = 0;
    }

    //--------------------------------------------------------------------------
    // Robot movement functions
    //--------------------------------------------------------------------------
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

        let expected_ticks = (distance_mm as f32 * self.config.wheel_ticks_per_mm) as u32;
        info!(
            "Robot move straight, distance = {}, target ticks = {}",
            distance_mm, expected_ticks
        );
        self.reset_wheel_counters();

        writeln!(
            self.logger,
            "STRAIGHT: distance = {} mm, forward = {}\ntarget ticks = {}\nmovement data = {{\n{}",
            distance_mm,
            forward,
            expected_ticks,
            StraightTelemetryRow::header().join(", "),
        )
        .ok();

        let mut traveled_ticks: u32 = 0;
        let mut last_update_millis = 0;
        let mut left_wheel_ticks = 0;
        let mut right_wheel_ticks = 0;

        let mut controller = PIDController::new(
            self.config.straight_pid_p,
            self.config.straight_pid_i,
            self.config.straight_pid_d,
        );
        // we want to go straight, so the setpoint is 0
        controller.set_setpoint(0.);
        self.heading_calculator.reset();

        self.motors.set_duty(
            self.config.straight_left_power,
            self.config.straight_right_power,
        );

        writeln!(
            self.logger,
            "{}",
            StraightTelemetryRow::new(
                millis(),
                0,
                0,
                100,
                100,
                self.heading_calculator.heading(),
                0.,
            ),
        )
        .ok();

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

                let heading = self.heading_calculator.heading();
                let control_signal = controller.update(heading, last_update_millis);

                let mut cs_indicator: &str = direction_arrow;
                // positive control signal means turn left, a negative control signal means turn right
                let mut left_power: f32 = self.config.straight_left_power as f32;
                let mut right_power: f32 = self.config.straight_right_power as f32;
                if (forward && control_signal > 0.) || (!forward && control_signal < 0.) {
                    left_power -= control_signal;
                    cs_indicator = LEFT_ARROW_STRING;
                } else if (forward && control_signal < 0.) || (!forward && control_signal > 0.) {
                    right_power += control_signal;
                    cs_indicator = RIGHT_ARROW_STRING;
                }
                left_power = left_power.clamp(0., 100.);
                right_power = right_power.clamp(0., 100.);

                self.motors.set_duty(left_power as u8, right_power as u8);

                writeln!(
                    self.logger,
                    "{}",
                    StraightTelemetryRow::new(
                        last_update_millis,
                        left_wheel_ticks,
                        right_wheel_ticks,
                        left_power as u8,
                        right_power as u8,
                        heading,
                        control_signal,
                    ),
                )
                .ok();
                self.heading_calculator.update();

                let wheel_ticks_per_mm = self.config.wheel_ticks_per_mm;
                if let Err(error) = core::write!(
                    self.set_lcd_cursor(0, 0),
                    "{} {} / {} mm",
                    direction_arrow,
                    (traveled_ticks as f32 / wheel_ticks_per_mm) as i32,
                    distance_mm,
                ) {
                    error!("Error writing to LCD: {}", error.to_string().as_str());
                }
                self.heading_calculator.update();
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

        writeln!(
            self.logger,
            "{}\n}}",
            StraightTelemetryRow::new(
                millis(),
                left_wheel_ticks,
                right_wheel_ticks,
                0,
                0,
                self.heading_calculator.heading(),
                0.,
            ),
        )
        .ok();
        self.logger.flush_buffer().ok();
        self.start_display_reset_timer();

        self
    }

    pub fn min_turn_angle(&self) -> f32 {
        1.0
    }

    pub fn turn(&mut self, angle_degrees: i32) -> &mut Self {
        const TURN_MIN_POWER: f32 = 0.50;
        if angle_degrees.abs() < self.min_turn_angle() as i32 {
            debug!("Turn angle {} too small, not turning", angle_degrees);
            return self;
        }

        info!("Robot turn, angle = {}", angle_degrees);
        self.reset_wheel_counters();
        let turn_degrees = if angle_degrees > 180 {
            angle_degrees - 360
        } else if angle_degrees < -180 {
            angle_degrees + 360
        } else {
            angle_degrees
        };

        let direction_str = if turn_degrees > 0 {
            LEFT_ARROW_STRING
        } else {
            RIGHT_ARROW_STRING
        };
        if let Err(error) = core::write!(
            self.clear_lcd().set_lcd_cursor(0, 0),
            "{} 0 / {}\x03",
            direction_str,
            turn_degrees,
        ) {
            error!("Error writing to LCD: {}", error.to_string().as_str());
        }
        self.heading_calculator.reset();
        let mut current_angle = self.heading_calculator.heading();
        let mut last_adjust_angle = current_angle;

        // start the motors per right hand rule (postive angle = left turn, negative angle = right turn)
        // motor A is the left motor, motor B is the right motor
        #[allow(unused_assignments)]
        let mut stop_angle_delta: i32 = 0;
        if turn_degrees > 0 {
            self.motors.set_duty(
                self.config.turn_left_left_power,
                self.config.turn_left_right_power,
            );
            self.motors.reverse_a();
            self.motors.forward_b();
            stop_angle_delta = self.config.turn_left_stop_angle_delta;
        } else {
            self.motors.set_duty(
                self.config.turn_right_left_power,
                self.config.turn_right_right_power,
            );
            self.motors.forward_a();
            self.motors.reverse_b();
            stop_angle_delta = self.config.turn_right_stop_angle_delta;
        };

        while current_angle.abs() < (turn_degrees.abs() + stop_angle_delta) as f32 {
            current_angle = self.heading_calculator.heading();
            if (current_angle - last_adjust_angle).abs() > 10.0 {
                last_adjust_angle = current_angle;
                if let Err(error) = core::write!(
                    self.clear_lcd().set_lcd_cursor(0, 0),
                    "{} {} / {}\x03",
                    direction_str,
                    current_angle as i32,
                    turn_degrees,
                ) {
                    error!("Error writing to LCD: {}", error.to_string().as_str());
                }
            }

            self.handle_loop();
        }
        let motors_off_heading = self.heading_calculator.heading();
        self.motors.stop();
        let start_millis = millis();
        while millis() - start_millis < 1000 {
            self.handle_loop();
        }
        current_angle = self.heading_calculator.heading();
        if let Err(error) = core::write!(
            self.clear_lcd().set_lcd_cursor(0, 0),
            "{} {} / {}\x03",
            direction_str,
            current_angle as i32,
            turn_degrees,
        ) {
            error!("Error writing to LCD: {}", error.to_string().as_str());
        }
        writeln!(
            self.logger,
            "TURN: target angle = {}, motors off angle = {}, final angle = {}",
            turn_degrees, motors_off_heading as i32, current_angle as i32
        )
        .ok();
        info!("Done with turn movement");
        self.start_display_reset_timer();
        self
    }

    pub fn stop(&mut self) -> &mut Self {
        self.motors.set_duty(0, 0);
        self
    }

    //--------------------------------------------------------------------------
    // Test functions
    //--------------------------------------------------------------------------
    pub fn display_heading(
        &mut self,
    ) -> Result<(), i2c_character_display::Error<CriticalSectionDevice<'a, TWI>>> {
        write!(self.lcd.clear()?.set_cursor(0, 0)?, "Heading:").ok();
        self.heading_calculator.reset();
        let mut continue_loop = true;

        let mut last_update_millis = 0;
        while continue_loop {
            self.handle_loop();
            if self.button1_pressed() || self.button2_pressed() {
                continue_loop = false;
            }
            if millis() - last_update_millis > 500 {
                let heading = self.heading_calculator.heading();
                if let Err(error) = core::write!(
                    self.lcd.set_cursor(0, 1)?,
                    "{:.2}{: <16}",
                    heading,
                    DEGREES_STRING
                ) {
                    error!("Error writing to LCD: {}", error.to_string().as_str());
                }
                last_update_millis = millis();
            }
        }

        Ok(())
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
        ENA: SetDutyCycle,
        ENB: SetDutyCycle,
        BUTT1: InputPin,
        BUTT2: InputPin,
        TWI,
        SPI_DEV: SpiDevice<u8>,
        DELAY,
    > Format for Robot<'a, INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, SPI_DEV, DELAY>
where
    TWI: I2c,
    DELAY: DelayNs + Clone,
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
        ENA: SetDutyCycle,
        ENB: SetDutyCycle,
        BUTT1: InputPin,
        BUTT2: InputPin,
        TWI,
        SPI_DEV: SpiDevice<u8>,
        DELAY,
    > core::fmt::Write
    for Robot<'a, INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, SPI_DEV, DELAY>
where
    TWI: I2c,
    DELAY: DelayNs + Clone,
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
