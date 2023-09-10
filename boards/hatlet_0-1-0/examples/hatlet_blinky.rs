//! # Pico Blinky Example
//!
//! Blinks the LED on a Pico board.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use core::convert::TryInto;

use hal::gpio::DynPinId;
use hal::gpio::FunctionI2C;
use hal::gpio::FunctionSio;
use hal::gpio::Pin;
use hal::gpio::PullDown;
use hal::gpio::PullNone;
use hal::gpio::SioOutput;
// The macro for our start-up function
use hatlet_0_1_0::entry;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

use heapless::pool::Box;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use hatlet_0_1_0::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hatlet_0_1_0::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use hatlet_0_1_0::hal;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        hatlet_0_1_0::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = hatlet_0_1_0::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set board LEDs as outputs
    let mut leds: [Pin<DynPinId, FunctionSio<SioOutput>, PullDown>; 2]  = [
        pins.led0.into_push_pull_output().into_dyn_pin(),
        pins.led1.into_push_pull_output().into_dyn_pin(),
    ];

    // Blink the LEDs at 1 Hz
    loop {
        for led in &mut leds {
            led.set_high().unwrap();
        }
        delay.delay_ms(500);
        for led in &mut leds {
            led.set_low().unwrap();
        }
        delay.delay_ms(500);
    }
}

// End of file
