#![feature(cell_update, asm_experimental_arch)]
#![no_std]
#![no_main]

extern crate alloc;

use esp_backtrace as _;
use esp_hal::{
    analog::adc::AdcChannel,
    clock::CpuClock,
    gpio::{GpioPin, Level, Output},
    mcpwm::timer::PeriodUpdatingMethod,
    peripheral,
};
use esp_hal::{delay::Delay, xtensa_lx_rt::entry};
use fugit::ExtU64;
use log::info;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);

    let peripherals: esp_hal::peripherals::Peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let delay = Delay::new();

    let mut gpio = Output::new(peripherals.GPIO20, Level::Low);

    loop {
        info!("Hello, world!");
        gpio.toggle();
        delay.delay(500.millis());
    }
}
