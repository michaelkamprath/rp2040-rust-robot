#![allow(non_camel_case_types, dead_code, clippy::type_complexity)]
use core::fmt::Write;

use crate::{model::point::Point, robot::Robot, system::millis::millis};
use defmt::{debug, warn};
use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    blocking::i2c::{Write as I2cWrite, WriteRead},
    digital::v2::{InputPin, OutputPin},
    PwmPin,
};
use micromath::F32Ext;

pub struct Driver<
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
    SPI,
    CS: OutputPin,
    DELAY,
    LED1: OutputPin,
> where
    TWI: I2cWrite<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8> + DelayUs<u8> + Copy,
{
    robot: Robot<'a, INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, TWI_ERR, SPI, CS, DELAY>,
    delay: DELAY,
    led1: LED1,
}

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
        SPI,
        CS: OutputPin,
        DELAY,
        LED1: OutputPin,
    > Driver<'a, INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI, TWI_ERR, SPI, CS, DELAY, LED1>
where
    TWI: I2cWrite<Error = TWI_ERR> + WriteRead<Error = TWI_ERR>,
    TWI_ERR: defmt::Format,
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
    DELAY: DelayMs<u16> + DelayUs<u16> + DelayMs<u8> + DelayUs<u8> + Copy,
{
    pub fn new(
        robot: Robot<
            'a,
            INA1,
            INA2,
            INB1,
            INB2,
            ENA,
            ENB,
            BUTT1,
            BUTT2,
            TWI,
            TWI_ERR,
            SPI,
            CS,
            DELAY,
        >,
        delay: DELAY,
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
            self.delay.delay_us(500_u16);
        }
    }

    pub fn handle_loop(&mut self) {
        if self.robot.button1_pressed() {
            self.led1.set_high().ok();
            self.trace_path(&[
                Point::new(0, 0),
                Point::new(0, 500),
                Point::new_with_forward(0, 0, false),
            ]);
            self.led1.set_low().ok();
        }
        if self.robot.button2_pressed() {
            debug!("button2 pressed");
            write!(
                self.robot.clear_lcd().set_lcd_cursor(0, 0),
                "button2 pressed",
            )
            .ok();
            write!(self.robot.set_lcd_cursor(0, 1), "ms: {}", millis()).ok();
            self.robot.start_display_reset_timer();
        }
        self.robot.handle_loop();
    }

    pub fn trace_path(&mut self, point_sequence: &[Point]) {
        if point_sequence.len() < 2 {
            warn!("trace_path: point_sequence too short");
            return;
        }
        let mut cur_point = point_sequence[0];
        // start with the bearing from the first point to the second point
        let mut cur_bearing: f32 = point_sequence[0].absolute_bearing(&point_sequence[1]);

        for next_point in point_sequence[1..].iter() {
            let distance = cur_point.distance_to(next_point);
            let bearing = cur_point.absolute_bearing(next_point);
            let bearing_diff = bearing - cur_bearing;
            debug!("driving to next way point\n  cur_point: {:?}, next_point: {:?}\n  distance: {}, bearing {}, forward: {}", cur_point, next_point, distance, bearing, next_point.forward());

            // turn to the correct bearing
            if bearing_diff.abs() > self.robot.min_turn_angle() {
                self.robot.turn(bearing_diff as i32);
            }

            // drive the distance
            self.robot.straight(distance as u32, next_point.forward());

            // update the current point and bearing
            cur_point = *next_point;
            cur_bearing = bearing;
        }
    }
}
