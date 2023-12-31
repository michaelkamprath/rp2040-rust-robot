use crate::{l298n::motor_controller::MotorController, model::heading::HeadingCalculator};
use alloc::rc::Rc;
use core::cell::{Cell, RefCell};
use cortex_m::{delay::Delay, interrupt::Mutex};
use defmt::{debug, info, trace};
use embedded_hal::{
    blocking::i2c::{Write, WriteRead},
    digital::v2::{InputPin, OutputPin},
    PwmPin,
};
use rp_pico::hal::gpio;
use rp_pico::hal::{
    gpio::{
        bank0::{Gpio16, Gpio17},
        FunctionSio, Interrupt, SioInput,
    },
    pac::{self, interrupt},
};

//==============================================================================
// Interrupt pins and counters for wheel encoders
//
type LeftWheelCounterPin = gpio::Pin<Gpio16, FunctionSio<SioInput>, gpio::PullUp>;
type RightWheelCounterPin = gpio::Pin<Gpio17, FunctionSio<SioInput>, gpio::PullUp>;

static LEFT_WHEEL_COUNTER_PIN: Mutex<RefCell<Option<LeftWheelCounterPin>>> =
    Mutex::new(RefCell::new(None));
static RIGHT_WHEEL_COUNTER_PIN: Mutex<RefCell<Option<RightWheelCounterPin>>> =
    Mutex::new(RefCell::new(None));

static LEFT_WHEEL_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
static RIGHT_WHEEL_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
//==============================================================================

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
> {
    motors: MotorController<INA1, INA2, INB1, INB2, ENA, ENB>,
    button1: BUTT1,
    button2: BUTT2,
    button1_pressed: bool,
    button2_pressed: bool,
    _i2c: Rc<RefCell<TWI>>,
    pub heading_calculator: HeadingCalculator<TWI>,
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
    > Robot<INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI>
where
    TWI: Write<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
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
        delay: &mut Delay,
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

        let mut heading_calculator = HeadingCalculator::new(&i2c_shared, delay);
        heading_calculator.reset();

        // return the robot
        info!("Robot initialized");
        Self {
            motors,
            button1: button1_pin,
            button2: button2_pin,
            button1_pressed: false,
            button2_pressed: false,
            _i2c: i2c_shared,
            heading_calculator,
        }
    }

    /// This function is called in the main loop to allow the robot to handle state updates
    pub fn handle_loop(&mut self) {
        // unset button press if button is not pressed
        if self.button1.is_high().ok().unwrap() {
            self.button1_pressed = false;
        }
        if self.button2.is_high().ok().unwrap() {
            self.button2_pressed = false;
        }

        self.heading_calculator.update();
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
        // the button is active low
        if self.button1.is_low().ok().unwrap() {
            if !self.button1_pressed {
                debug!("robot button 1 pressed");
                self.button1_pressed = true;
                return true;
            }
        } else {
            self.button1_pressed = false;
        }
        false
    }

    /// returns true if the button 2 is newly pressed
    pub fn button2_pressed(&mut self) -> bool {
        // the button is active low
        if self.button2.is_low().ok().unwrap() {
            if !self.button2_pressed {
                debug!("robot button 2 pressed");
                self.button2_pressed = true;
                return true;
            }
        } else {
            self.button2_pressed = false;
        }
        false
    }

    pub fn forward(&mut self, duty: f32) -> &mut Self {
        let normalized_duty = self.noramlize_duty(duty);
        self.motors.set_duty(normalized_duty / 10, normalized_duty);
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
                trace!("Left wheel count: {}", LEFT_WHEEL_COUNTER.borrow(cs).get());
            }
        }
        if let Some(pin) = RIGHT_WHEEL_COUNTER_PIN.borrow(cs).borrow_mut().as_mut() {
            if pin.interrupt_status(Interrupt::EdgeLow) {
                RIGHT_WHEEL_COUNTER
                    .borrow(cs)
                    .set(RIGHT_WHEEL_COUNTER.borrow(cs).get() + 1);
                pin.clear_interrupt(Interrupt::EdgeLow);
                trace!(
                    "Right wheel count: {}",
                    RIGHT_WHEEL_COUNTER.borrow(cs).get()
                );
            }
        }
    });
}
