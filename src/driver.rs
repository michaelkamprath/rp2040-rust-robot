#![allow(non_camel_case_types, dead_code)]

use core::{cell::RefCell, fmt::Write};

use crate::{robot::Robot, system::millis::millis};
use alloc::rc::Rc;
use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    blocking::i2c::{Write as I2cWrite, WriteRead},
    digital::v2::{InputPin, OutputPin},
    PwmPin,
};

pub struct Driver<
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
    LED1: OutputPin,
> {
    robot: Robot<INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, DELAY>,
    delay: Rc<RefCell<DELAY>>,
    led1: LED1,
}

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
        LED1: OutputPin,
    > Driver<INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, DELAY, LED1>
where
    TWI: I2cWrite<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8>,
{
    pub fn new(
        robot: Robot<INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, DELAY>,
        delay: Rc<RefCell<DELAY>>,
        led1_pin: LED1,
    ) -> Self {
        Self {
            robot,
            delay,
            led1: led1_pin,
        }
    }

    /// Delays for the given milliseconds, calling the robot's loop handler often. Ignores button presses.
    pub fn delay(&mut self, ms: u32) {
        // we are going to call the loop handler in the delay function every ms until the delay is complete
        let start_millis = millis();
        while millis() - start_millis < ms {
            self.robot.handle_loop();
            self.delay.borrow_mut().delay_us(500);
        }
    }

    pub fn handle_loop(&mut self) {
        if self.robot.button1_pressed() {
            self.led1.set_high().ok();
            self.robot.straight(1000);
            self.delay(1000);
            self.robot.stop();
            self.led1.set_low().ok();
            write!(self.robot.clear_lcd().set_lcd_cursor(0, 0), "done",).ok();
        }
        if self.robot.button2_pressed() {
            write!(
                self.robot.clear_lcd().set_lcd_cursor(0, 0),
                "button2 pressed",
            )
            .ok();
            write!(self.robot.set_lcd_cursor(0, 1), "ms: {}", millis()).ok();
        }
        self.robot.handle_loop();
    }
}
