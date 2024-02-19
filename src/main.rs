#![no_std]
#![no_main]
mod driver;
mod model;
mod robot;
mod system;

use alloc::rc::Rc;
use bsp::{
    entry,
    hal::{fugit::HertzU32, gpio},
};
use core::cell::RefCell;
use defmt::{info, panic};
use defmt_rtt as _;
use panic_probe as _;
use rp2040_hal::{
    gpio::{
        bank0::{Gpio0, Gpio2, Gpio3},
        FunctionSpi, PullDown,
    },
    pac::SPI0,
};

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

use driver::Driver;
use robot::Robot;
use system::millis::init_millis;

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

    // set up SPI
    let spi: rp_pico::hal::Spi<
        rp_pico::hal::spi::Disabled,
        SPI0,
        (
            rp_pico::hal::gpio::Pin<Gpio3, FunctionSpi, PullDown>,
            rp_pico::hal::gpio::Pin<Gpio0, FunctionSpi, PullDown>,
            rp_pico::hal::gpio::Pin<Gpio2, FunctionSpi, PullDown>,
        ),
    > = bsp::hal::Spi::new(
        pac.SPI0,
        (
            pins.gpio3.into_function::<gpio::FunctionSpi>(),
            pins.gpio0.into_function::<gpio::FunctionSpi>(),
            pins.gpio2.into_function::<gpio::FunctionSpi>(),
        ),
    );

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        HertzU32::from_raw(400_000), // card initialization happens at low baud rate
        embedded_hal::spi::MODE_0,
    );

    let sd = crate::robot::file_storage::FileStorage::new(
        spi,
        pins.gpio1.into_push_pull_output(),
        timer.clone(),
    );

    let robot = Robot::new(
        pins.gpio10.into_push_pull_output(),
        pins.gpio11.into_push_pull_output(),
        pins.gpio13.into_push_pull_output(),
        pins.gpio12.into_push_pull_output(),
        channel_a,
        channel_b,
        pins.gpio14.into_pull_up_input(),
        pins.gpio15.into_pull_up_input(),
        i2c,
        pins.gpio21.into_pull_up_input(),
        pins.gpio20.into_pull_up_input(),
        &mut delay_shared,
    );

    let mut driver = Driver::new(
        robot,
        delay_shared.clone(),
        pins.led.into_push_pull_output(),
    );

    info!("robot controller and driver created");

    loop {
        driver.handle_loop();
    }
}
