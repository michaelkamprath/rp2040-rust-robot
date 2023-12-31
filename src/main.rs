#![no_std]
#![no_main]
mod l298n;
mod model;
mod robot;
mod system;

use bsp::{
    entry,
    hal::{fugit::HertzU32, gpio},
};
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

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
    let mut pwm3 = pwm_slices.pwm3;

    pwm3.set_ph_correct();
    pwm3.enable();

    let mut channel_a = pwm3.channel_a;
    let mut channel_b = pwm3.channel_b;

    channel_a.output_to(pins.gpio6);
    channel_b.output_to(pins.gpio7);

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
        pins.gpio8.into_push_pull_output(),
        pins.gpio9.into_push_pull_output(),
        pins.gpio10.into_push_pull_output(),
        pins.gpio11.into_push_pull_output(),
        channel_a,
        channel_b,
        pins.gpio0.into_pull_up_input(),
        pins.gpio1.into_pull_up_input(),
        i2c,
        pins.gpio16.into_pull_up_input(),
        pins.gpio17.into_pull_up_input(),
        &mut delay,
    );

    info!("robot controller created");

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let mut led_pin = pins.led.into_push_pull_output();
    let mut test_pin = pins.gpio2.into_push_pull_output();

    loop {
        info!(
            "on! millis: {}, heading: {}",
            millis(),
            robot.heading_calculator.heading()
        );
        robot.forward(1.0);
        led_pin.set_high().unwrap();
        test_pin.set_high().unwrap();
        delay.delay_ms(250);
        robot.handle_loop();
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
        robot.handle_loop();
        led_pin.set_high().unwrap();
        delay.delay_ms(250);
        robot.handle_loop();
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
        robot.handle_loop();
        led_pin.set_high().unwrap();
        delay.delay_ms(250);
        robot.handle_loop();
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
        robot.handle_loop();
        led_pin.set_high().unwrap();
        delay.delay_ms(250);
        robot.handle_loop();

        info!("off!");
        robot.stop();
        led_pin.set_low().unwrap();
        test_pin.set_low().unwrap();
        delay.delay_ms(1000);

        robot.handle_loop();
    }
}
