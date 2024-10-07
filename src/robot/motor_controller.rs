use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

/// A L298N motor controller that can control two motors.
pub struct MotorController<INA1, INA2, INB1, INB2, ENA, ENB> {
    ina1: INA1,
    ina2: INA2,
    inb1: INB1,
    inb2: INB2,
    ena: ENA,
    enb: ENB,
    duty_a: u8,
    duty_b: u8,
}

#[allow(dead_code)]
impl<INA1, INA2, INB1, INB2, ENA, ENB> MotorController<INA1, INA2, INB1, INB2, ENA, ENB>
where
    INA1: OutputPin,
    INA2: OutputPin,
    INB1: OutputPin,
    INB2: OutputPin,
    ENA: SetDutyCycle,
    ENB: SetDutyCycle,
{
    pub fn new(ina1: INA1, ina2: INA2, inb1: INB1, inb2: INB2, ena: ENA, enb: ENB) -> Self
    where
        INA1: OutputPin,
        INA2: OutputPin,
        INB1: OutputPin,
        INB2: OutputPin,
        ENA: SetDutyCycle,
        ENB: SetDutyCycle,
    {
        Self {
            ina1,
            ina2,
            inb1,
            inb2,
            ena,
            enb,
            duty_a: 0,
            duty_b: 0,
        }
    }

    // pub fn enable_pin_a(&self) -> &ENA {
    //     &self.ena
    // }

    // pub fn enable_pin_b(&self) -> &ENB {
    //     &self.enb
    // }

    pub fn set_duty(&mut self, duty_percent_a: u8, duty_percent_b: u8) {
        self.set_duty_a(duty_percent_a);
        self.set_duty_b(duty_percent_b);
    }

    pub fn set_duty_a(&mut self, duty_percent_a: u8) {
        self.duty_a = duty_percent_a;
        if self.duty_a > 100 {
            self.duty_a = 100;
        }
    }

    pub fn get_duty_percent_a(&self) -> u8 {
        self.duty_a
    }

    pub fn enable_a(&mut self) {
        let _ = self.ena.set_duty_cycle_percent(self.duty_a);
    }

    pub fn disable_a(&mut self) {
        let _ = self.ena.set_duty_cycle_percent(0);
    }

    pub fn set_duty_b(&mut self, duty_percent_b: u8) {
        self.duty_b = duty_percent_b;
        if self.duty_b > 100 {
            self.duty_b = 100;
        }
        let _ = self.enb.set_duty_cycle_percent(self.duty_b);
    }

    pub fn get_duty_percent_b(&self) -> u8 {
        self.duty_b
    }

    pub fn enable_b(&mut self) {
        let _ = self.enb.set_duty_cycle_percent(self.duty_b);
    }

    pub fn disable_b(&mut self) {
        let _ = self.enb.set_duty_cycle_percent(0);
    }

    pub fn forward(&mut self) {
        self.ina1.set_high().ok();
        self.ina2.set_low().ok();
        self.inb1.set_high().ok();
        self.inb2.set_low().ok();
        self.enable_a();
        self.enable_b();
    }

    pub fn forward_a(&mut self) {
        self.ina1.set_high().ok();
        self.ina2.set_low().ok();
        self.enable_a();
    }

    pub fn forward_b(&mut self) {
        self.inb1.set_high().ok();
        self.inb2.set_low().ok();
        self.enable_b();
    }

    pub fn reverse(&mut self) {
        self.ina1.set_low().ok();
        self.ina2.set_high().ok();
        self.inb1.set_low().ok();
        self.inb2.set_high().ok();
        self.enable_a();
        self.enable_b();
    }

    pub fn reverse_a(&mut self) {
        self.ina1.set_low().ok();
        self.ina2.set_high().ok();
        self.enable_a();
    }

    pub fn reverse_b(&mut self) {
        self.inb1.set_low().ok();
        self.inb2.set_high().ok();
        self.enable_b();
    }

    pub fn stop(&mut self) {
        self.disable_a();
        self.disable_b();
        self.ina1.set_low().ok();
        self.ina2.set_low().ok();
        self.inb1.set_low().ok();
        self.inb2.set_low().ok();
    }

    pub fn stop_a(&mut self) {
        self.disable_a();
        self.ina1.set_low().ok();
        self.ina2.set_low().ok();
    }

    pub fn stop_b(&mut self) {
        self.disable_b();
        self.inb1.set_low().ok();
        self.inb2.set_low().ok();
    }

    /// Break the motor controller to stop the motors quickly.
    /// This will set the duty cycle to 100% and set the direction pins to high.
    /// `stop` should be called after calling `brake` and before calling `forward` to stop the motors.
    pub fn brake(&mut self) {
        self.ina1.set_high().ok();
        self.ina2.set_high().ok();
        self.inb1.set_high().ok();
        self.inb2.set_high().ok();
        let _ = self.ena.set_duty_cycle_percent(100);
        let _ = self.enb.set_duty_cycle_percent(100);
    }
}
