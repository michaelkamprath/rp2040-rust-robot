#![no_std]
#![no_main]
mod l298n;
mod model;
mod robot;
mod system;

use alloc::rc::Rc;
use bsp::{
    entry,
    hal::{fugit::HertzU32, gpio},
};
use core::{cell::RefCell, fmt::Write};
use defmt::{info, panic};
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

extern crate alloc;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    pwm::Slices,
    sio::Sio,
    watchdog::Watchdog,
};

use robot::Robot;
use system::millis::{init_millis, millis};

#[entry]
fn main() -> ! {
    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 4048;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

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

    let mut delay_shared = Rc::new(RefCell::new(cortex_m::delay::Delay::new(
        core.SYST,
        clocks.system_clock.freq().to_Hz(),
    )));

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // init timer and millis
    let mut timer = rp_pico::hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    if let Err(_e) = init_millis(&mut timer) {
        panic!("Error initializing millis");
    }

    // Init & Configure PWMs
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm4 = pwm_slices.pwm4;

    pwm4.set_ph_correct();
    pwm4.enable();

    let mut channel_a = pwm4.channel_a;
    let mut channel_b = pwm4.channel_b;

    channel_a.output_to(pins.gpio8);
    channel_b.output_to(pins.gpio9);

    // set up I2C
    let i2c = bsp::hal::I2C::new_controller(
        pac.I2C0,
        pins.gpio4.into_function::<gpio::FunctionI2c>(),
        pins.gpio5.into_function::<gpio::FunctionI2c>(),
        HertzU32::from_raw(400_000),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let mut robot = Robot::new(
        pins.gpio10.into_push_pull_output(),
        pins.gpio11.into_push_pull_output(),
        pins.gpio13.into_push_pull_output(),
        pins.gpio12.into_push_pull_output(),
        channel_a,
        channel_b,
        pins.gpio18.into_pull_up_input(),
        pins.gpio19.into_pull_up_input(),
        i2c,
        pins.gpio16.into_pull_up_input(),
        pins.gpio22.into_pull_up_input(),
        &mut delay_shared,
    );

    info!("robot controller created");

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        if robot.button1_pressed() {
            info!("button1 pressed");
            robot.reset_wheel_counters();
            let heading = robot.heading_calculator.heading();
            write!(
                robot.clear_lcd().set_lcd_cursor(0, 0),
                "heading: {}",
                heading,
            )
            .ok();
            robot.forward(1.0);
            led_pin.set_high().unwrap();
            loop_delay(&mut delay_shared, 2000, || {
                robot.handle_loop();
            });

            robot.stop();
            led_pin.set_low().unwrap();
            loop_delay(&mut delay_shared, 500, || {
                robot.handle_loop();
            });
            info!("off ===> {}", robot);
            let left_wheel_counter = robot.get_left_wheel_counter();
            let right_wheel_counter = robot.get_right_wheel_counter();
            write!(
                robot.set_lcd_cursor(0, 1),
                "l: {} r: {}",
                left_wheel_counter,
                right_wheel_counter,
            )
            .ok();
            loop_delay(&mut delay_shared, 1000, || {
                robot.handle_loop();
            });

            robot.handle_loop();
        }
        if robot.button2_pressed() {
            write!(robot.clear_lcd().set_lcd_cursor(0, 0), "button2 pressed",).ok();
            write!(robot.set_lcd_cursor(0, 1), "millis: {}", millis()).ok();
            delay_shared.borrow_mut().delay_ms(500);
        }
        robot.handle_loop();
    }
}

fn loop_delay<F>(
    delayer: &mut Rc<RefCell<impl embedded_hal::blocking::delay::DelayMs<u32>>>,
    delay_ms: u32,
    mut f: F,
) where
    F: FnMut(),
{
    // we want to call f() at least every 25 ms
    let start_millis = millis();
    while millis() - start_millis < delay_ms {
        f();
        let next_loop_delay = (delay_ms - (millis() - start_millis)).max(1).min(25);
        delayer.borrow_mut().delay_ms(next_loop_delay);
    }
}
