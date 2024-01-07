#![allow(non_camel_case_types, dead_code)]
mod debouncer;

use crate::{
    l298n::motor_controller::MotorController, model::heading::HeadingCalculator,
    robot::debouncer::DebouncedButton,
};
use adafruit_lcd_backpack::{LcdBackpack, LcdDisplayType};
use alloc::rc::Rc;
use core::cell::{Cell, RefCell};
use cortex_m::interrupt::Mutex;
use defmt::{error, info};
use defmt::{write, Format};
use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    blocking::i2c::{Write, WriteRead},
    digital::v2::{InputPin, OutputPin},
    PwmPin,
};
use rp_pico::hal::gpio;
use rp_pico::hal::{
    gpio::{
        bank0::{Gpio16, Gpio22},
        FunctionSio, Interrupt, SioInput,
    },
    pac::{self, interrupt},
};

//==============================================================================
// Interrupt pins and counters for wheel encoders
//
type LeftWheelCounterPin = gpio::Pin<Gpio16, FunctionSio<SioInput>, gpio::PullUp>;
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
const WHEEL_DIAMETER: f32 = 65.0; // mm
const WHEEL_CIRCUMFERENCE: f32 = WHEEL_DIAMETER * core::f32::consts::PI;
const MOTOR_REDUCTION_RATIO: f32 = 46.8;
const ENCODER_TICKS_PER_REVOLUTION: f32 = 11.0;
const WHEEL_TICKS_PER_REVOLUTION: f32 = ENCODER_TICKS_PER_REVOLUTION * MOTOR_REDUCTION_RATIO;
const WHEEL_TICKS_PER_MM: f32 = WHEEL_TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

const BUTTON_DEBOUNCE_TIME_MS: u32 = 10;
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
    TWI: Write<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
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
        lcd.print("Hellorld!").ok();

        // return the robot
        info!("Robot initialized");
        Self {
            motors,
            button1: DebouncedButton::<BUTT1, false, BUTTON_DEBOUNCE_TIME_MS>::new(button1_pin),
            button2: DebouncedButton::<BUTT2, false, BUTTON_DEBOUNCE_TIME_MS>::new(button2_pin),
            _i2c: i2c_shared,
            heading_calculator,
            lcd,
        }
    }

    /// This function is called in the main loop to allow the robot to handle state updates
    pub fn handle_loop(&mut self) {
        // unset button press if button is not pressed
        self.button1.handle_loop();
        self.button2.handle_loop();

        // self.heading_calculator.update();
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

    pub fn forward(&mut self, duty: f32) -> &mut Self {
        let normalized_duty = self.noramlize_duty(duty);
        self.motors.set_duty(normalized_duty, normalized_duty);
        self.motors.forward();
        self
    }

    pub fn straight(&mut self, distance_mm: u32) -> &mut Self {
        info!("Robot move straight, distance = {}", distance_mm);

        self
    }

    pub fn stop(&mut self) -> &mut Self {
        self.motors.set_duty(0, 0);
        self
    }

    //--------------------------------------------------------------------------
    // LCD functions

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
    TWI: Write<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8>,
{
    fn format(&self, f: defmt::Formatter) {
        write!(
            f,
            "Robot< left = {}, right = {} >",
            self.get_left_wheel_counter(),
            self.get_right_wheel_counter()
        );
    }
}

/// Implement the `core::fmt::Write` trait for the robot, allowing us to use the `write!` macro
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
    TWI: Write<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8>,
{
    fn write_str(&mut self, s: &str) -> Result<(), core::fmt::Error> {
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
