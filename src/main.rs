#![no_std]
#![no_main]
mod driver;
mod model;
mod robot;
mod system;

use core::cell::RefCell;
use defmt::{error, info, panic};
use defmt_rtt as _;
use panic_probe as _;

use crate::robot::Robot;
use bsp::{
    entry,
    hal::fugit::HertzU32,
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{FunctionI2C, Pin, PullUp},
        pac,
        pwm::Slices,
        sio::Sio,
        watchdog::Watchdog,
    },
};
use driver::Driver;
use rp_pico as bsp;
use system::millis::init_millis;
extern crate alloc;

use embedded_alloc::TlsfHeap as Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

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
    let _core = pac::CorePeripherals::take().unwrap();
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

    // Configure two pins as being I²C, not GPIO
    let sda0_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio4.reconfigure();
    let scl0_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio5.reconfigure();
    let sda1_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio6.reconfigure();
    let scl1_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio7.reconfigure();
    // set up I2C
    let i2c0 = bsp::hal::I2C::new_controller(
        pac.I2C0,
        sda0_pin,
        scl0_pin,
        HertzU32::from_raw(400_000),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );
    let i2c0_ref_cell = RefCell::new(i2c0);
    let i2c0_mutex = critical_section::Mutex::new(i2c0_ref_cell);

    let i2c1 = bsp::hal::I2C::new_controller(
        pac.I2C1,
        sda1_pin,
        scl1_pin,
        HertzU32::from_raw(400_000),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );
    let i2c1_ref_cell = RefCell::new(i2c1);
    let i2c1_mutex = critical_section::Mutex::new(i2c1_ref_cell);

    // set up SPI
    #[allow(clippy::type_complexity)]
    let spi_mosi = pins.gpio3.into_function::<bsp::hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio0.into_function::<bsp::hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio2.into_function::<bsp::hal::gpio::FunctionSpi>();
    let spi = bsp::hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        HertzU32::from_raw(400_000), // card initialization happens at low baud rate
        embedded_hal::spi::MODE_0,
    );

    let sd = crate::robot::file_storage::FileStorage::new(
        crate::robot::file_storage::sd_card_spi_device::SDCardSPIDevice::new(
            spi,
            pins.gpio1.into_push_pull_output(),
            timer,
        ),
        timer,
    );

    // If SD card is successfully initialized, we can increase the SPI speed
    match sd.spi(|spi| {
        spi.bus.set_baudrate(
            clocks.peripheral_clock.freq(),
            HertzU32::from_raw(16_000_000),
        )
    }) {
        Some(speed) => {
            info!("SPI speed increased to {}", speed.raw());
        }
        None => {
            error!("Error increasing SPI speed");
        }
    }

    let mut robot = Robot::new(
        pins.gpio10.into_push_pull_output(),
        pins.gpio11.into_push_pull_output(),
        pins.gpio13.into_push_pull_output(),
        pins.gpio12.into_push_pull_output(),
        channel_a,
        channel_b,
        pins.gpio14.into_pull_up_input(),
        pins.gpio15.into_pull_up_input(),
        &i2c0_mutex,
        &i2c1_mutex,
        pins.gpio21.into_pull_up_input(),
        pins.gpio20.into_pull_up_input(),
        sd,
        &mut timer,
    );

    if let Err(e) = robot.init() {
        panic!("Error initializing SD card: {:?}", e);
    }

    let mut driver = Driver::new(robot, timer, pins.led.into_push_pull_output());

    info!("robot controller and driver created");

    loop {
        driver.handle_loop();
    }
}
