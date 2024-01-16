#![allow(non_camel_case_types, dead_code)]
mod debouncer;
mod motor_controller;
mod telemetry;

use crate::{
    model::{heading::HeadingCalculator, pid_controller::PIDController},
    robot::{
        debouncer::DebouncedButton, motor_controller::MotorController,
        telemetry::straight_movement::StraightTelemetryRow,
    },
    system::{data::DataTable, millis::millis},
};
use adafruit_lcd_backpack::{LcdBackpack, LcdDisplayType};
use alloc::{rc::Rc, string::ToString};
use core::{
    self,
    cell::{Cell, RefCell},
    fmt::Write,
};
use cortex_m::interrupt::Mutex;
use defmt::{debug, error, info, Format};
use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    blocking::i2c::{Write as I2cWrite, WriteRead},
    digital::v2::{InputPin, OutputPin},
    PwmPin,
};
use rp_pico::hal::gpio;
use rp_pico::hal::{
    gpio::{
        bank0::{Gpio21, Gpio22},
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

const UP_ARROW_STRING: &str = "\x01";
const DOWN_ARROW_STRING: &str = "\x02";
const LEFT_ARROW_STRING: &str = "\x7F";
const RIGHT_ARROW_STRING: &str = "\x7E";

const DISPLAY_RESET_DELAY_MS: u32 = 5000;

//==============================================================================
// Interrupt pins and counters for wheel encoders
//
type LeftWheelCounterPin = gpio::Pin<Gpio21, FunctionSio<SioInput>, gpio::PullUp>;
type RightWheelCounterPin = gpio::Pin<Gpio22, FunctionSio<SioInput>, gpio::PullUp>;

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
    INA1: OutputPin,
    INA2: OutputPin,
    INB1: OutputPin,
    INB2: OutputPin,
    ENA: PwmPin<Duty = u16>,
    ENB: PwmPin<Duty = u16>,
    BUTT1: InputPin,
    BUTT2: InputPin,
    TWI,
    DELAY,
> {
    motors: MotorController<INA1, INA2, INB1, INB2, ENA, ENB>,
    button1: DebouncedButton<BUTT1, false, BUTTON_DEBOUNCE_TIME_MS>,
    button2: DebouncedButton<BUTT2, false, BUTTON_DEBOUNCE_TIME_MS>,
    _i2c: Rc<RefCell<TWI>>,
    pub heading_calculator: HeadingCalculator<TWI, DELAY>,
    lcd: LcdBackpack<TWI, DELAY>,
    reset_display_start_millis: u32,
}

#[allow(dead_code, non_camel_case_types)]
impl<
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
        DELAY,
    > Robot<INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, DELAY>
where
    TWI: I2cWrite<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8>,
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
        i2c: TWI,
        left_counter_pin: LeftWheelCounterPin,
        right_counter_pin: RightWheelCounterPin,
        delay_shared: &mut Rc<RefCell<DELAY>>,
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
        let i2c_shared = Rc::new(RefCell::new(i2c));

        let mut heading_calculator = HeadingCalculator::new(&i2c_shared, delay_shared);
        heading_calculator.reset();

        let mut lcd = LcdBackpack::new(LcdDisplayType::Lcd16x2, &i2c_shared, delay_shared);
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

        if let Err(error) = lcd
            .home()
            .and_then(LcdBackpack::clear)
            .and_then(|lcd| LcdBackpack::print(lcd, "Robot Started"))
        {
            error!("Error writing to LCD: {}", error);
        }

        // return the robot
        info!("Robot initialized");
        Self {
            motors,
            button1: DebouncedButton::<BUTT1, false, BUTTON_DEBOUNCE_TIME_MS>::new(button1_pin),
            button2: DebouncedButton::<BUTT2, false, BUTTON_DEBOUNCE_TIME_MS>::new(button2_pin),
            _i2c: i2c_shared,
            heading_calculator,
            lcd,
            reset_display_start_millis: millis(), // start the display reset timer to clear the "started" message
        }
    }

    /// This function is called in the main loop to allow the robot to handle state updates
    pub fn handle_loop(&mut self) {
        if self.reset_display_start_millis != 0
            && millis() - self.reset_display_start_millis > DISPLAY_RESET_DELAY_MS
        {
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

    pub fn straight(&mut self, distance_mm: u32) -> &mut Self {
        if let Err(error) = core::write!(
            self.clear_lcd().set_lcd_cursor(0, 0),
            "{} 0 / {} mm",
            UP_ARROW_STRING,
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
        self.motors.forward();

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

                let mut cs_indicator: &str = UP_ARROW_STRING;
                // positive control signal means turn left, a negative control signal means turn right
                let mut left_power: f32 = 1.;
                let mut right_power: f32 = 1.;
                if control_signal > 0. {
                    left_power = 1. - control_signal;
                    right_power = 1.0;
                    cs_indicator = LEFT_ARROW_STRING;
                } else if control_signal < 0. {
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
                    UP_ARROW_STRING,
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

    pub fn turn(&mut self, angle_degrees: i32) -> &mut Self {
        info!("Robot turn, angle = {}", angle_degrees);
        self.reset_wheel_counters();

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
        DELAY,
    > Format for Robot<INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, DELAY>
where
    TWI: I2cWrite<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8>,
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
        DELAY,
    > core::fmt::Write for Robot<INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, DELAY>
where
    TWI: I2cWrite<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8>,
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
