//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
mod l298n;

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{digital::v2::OutputPin, PwmPin};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    pwm::Slices,
    sio::Sio,
    watchdog::Watchdog,
};

use l298n::motor_controller::MotorController;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init PWMs
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    // Configure PWM
    let mut pwm3 = pwm_slices.pwm3;

    pwm3.set_ph_correct();
    pwm3.enable();

    let mut channel_a = pwm3.channel_a;
    let mut channel_b = pwm3.channel_b;

    channel_a.output_to(pins.gpio6);
    channel_b.output_to(pins.gpio7);

    // set up motor controller
    let mut motor_controller = MotorController::new(
        pins.gpio8.into_push_pull_output(),
        pins.gpio9.into_push_pull_output(),
        pins.gpio10.into_push_pull_output(),
        pins.gpio11.into_push_pull_output(),
        channel_a,
        channel_b,
    );
    motor_controller.set_duty(
        (0.01 * motor_controller.enable_pin_a().get_max_duty() as f32) as u16,
        (1.0 * motor_controller.enable_pin_b().get_max_duty() as f32) as u16,
    );
    info!("motor_controller created");

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        info!("on!");
        motor_controller.forward();
        led_pin.set_high().unwrap();
        delay.delay_ms(250);
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
        led_pin.set_high().unwrap();
        delay.delay_ms(250);
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
        led_pin.set_high().unwrap();
        delay.delay_ms(250);
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
        led_pin.set_high().unwrap();
        delay.delay_ms(250);
        info!("off!");
        motor_controller.stop();
        led_pin.set_low().unwrap();
        delay.delay_ms(1000);
    }
}

// End of file
