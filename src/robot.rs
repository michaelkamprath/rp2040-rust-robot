use crate::l298n::motor_controller::MotorController;
use defmt::info;
use embedded_hal::{
    blocking::i2c::WriteRead,
    digital::v2::{InputPin, OutputPin},
    PwmPin,
};

pub struct Robot<
    INA1: OutputPin,
    INA2: OutputPin,
    INB1: OutputPin,
    INB2: OutputPin,
    ENA: PwmPin<Duty = u16>,
    ENB: PwmPin<Duty = u16>,
    BUTT1: InputPin,
    BUTT2: InputPin,
    TWI: WriteRead,
> {
    motors: MotorController<INA1, INA2, INB1, INB2, ENA, ENB>,
    _i2c: TWI,
    button1: BUTT1,
    button2: BUTT2,
    button1_pressed: bool,
    button2_pressed: bool,
}

#[allow(dead_code)]
impl<
        INA1: OutputPin,
        INA2: OutputPin,
        INB1: OutputPin,
        INB2: OutputPin,
        ENA: PwmPin<Duty = u16>,
        ENB: PwmPin<Duty = u16>,
        BUTT1: InputPin,
        BUTT2: InputPin,
        TWI: WriteRead,
    > Robot<INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI>
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
    ) -> Self {
        let motors = MotorController::new(ina1_pin, ina2_pin, inb1_pin, inb2_pin, ena_pin, enb_pin);

        Self {
            motors,
            _i2c: i2c,
            button1: button1_pin,
            button2: button2_pin,
            button1_pressed: false,
            button2_pressed: false,
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
                info!("robot button 1 pressed");
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
                info!("robot button 2 pressed");
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
        self.motors.set_duty(normalized_duty, normalized_duty);
        self.motors.forward();
        self
    }

    pub fn straight(&mut self, _distance_mm: u32) -> &mut Self {
        self
    }

    pub fn stop(&mut self) -> &mut Self {
        self.motors.set_duty(0, 0);
        self
    }
}
