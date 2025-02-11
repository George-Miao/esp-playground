#![feature(cell_update, asm_experimental_arch)]
#![no_std]
#![no_main]

extern crate alloc;

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    i2c::master::{Config, I2c},
};
use esp_hal::{
    gpio::{Level, Output},
    xtensa_lx_rt::entry,
};
use log::info;
use tap::Pipe;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);

    let peripherals: esp_hal::peripherals::Peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let mut led = Output::new(peripherals.GPIO4, Level::Low);
    let mut mag = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_scl(peripherals.GPIO18)
        .with_sda(peripherals.GPIO17)
        .pipe(as5600::As5600::new);

    let mut prev = 0;
    let start = mag.angle();
    let delay = esp_hal::delay::Delay::new();

    loop {
        delay.delay_millis(1);
        let curr = mag.angle().unwrap();
        if curr > prev && curr - prev > 4000 {
            info!("-= 1");
        }
        if curr < prev && prev - curr > 4000 {
            info!("+= 1");
        }
        if curr.abs_diff(prev) <= 1 {
            led.set_low();
            continue;
        }
        led.set_high();
        prev = curr;
        let real_angle = curr as f32 * 360.0 / 4096.0;
        info!("{real_angle:.2}° - {curr}");
    }
}
