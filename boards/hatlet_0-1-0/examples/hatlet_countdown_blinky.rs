//! # Pico Countdown Blinky Example
//!
//! Blinks the LED on a Pico board, using an RP2040 Timer in Count-down mode.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use hal::gpio::{PullDown, SioOutput, FunctionSio};
// The macro for our start-up function
use hatlet_0_1_0::entry;

use cortex_m::prelude::*;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Traits for converting integers to amounts of time
use fugit::ExtU32;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hatlet_0_1_0::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use hatlet_0_1_0::hal;

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

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

    // Configure the Timer peripheral in count-down mode
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut count_down = timer.count_down();

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
    let mut leds = [
        pins.led0.into_push_pull_output().into_dyn_pin(),
        pins.led1.into_push_pull_output().into_dyn_pin(),
    ];

    // Blink the LED at 1 Hz
    loop {
        // LED on, and wait for 500ms
        for led in &mut leds {
            led.set_high().unwrap();
        }
        count_down.start(500.millis());
        let _ = nb::block!(count_down.wait());
        
        // LED off, and wait for 500ms
        for led in &mut leds {
            led.set_low().unwrap();
        }
        count_down.start(500.millis());
        let _ = nb::block!(count_down.wait());
    }
}