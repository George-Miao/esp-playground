#![feature(cell_update, asm_experimental_arch)]
#![no_std]
#![no_main]

extern crate alloc;

use core::f32::consts::PI;

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    i2c::master::{Config, I2c},
    xtensa_lx_rt::entry,
};
use log::info;
use playground::sensor::Sensor;
use tap::Pipe;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);
    let peripherals: esp_hal::peripherals::Peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let encoder = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_scl(peripherals.GPIO12)
        .with_sda(peripherals.GPIO11)
        .pipe(as5600::As5600::new);

    let mut sensor = Sensor::new(encoder);
    let mut tick: u64 = 0;
    let step = PI / 2.;

    loop {
        tick += 1;

        sensor.update().unwrap();
        let state = sensor.state();

        let total_angle = state.total_angle();

        // Find the nearest step
        let mut error = -state.angle() % step;
        if error.abs() > 0.5 * step {
            error += -1. * error.signum() * step;
        }

        if tick % 100 == 0 {
            // log::info!(
            //     "{:.2} --({:.2})--> {:.2}",
            //     total_angle / (2. * PI),
            //     error / (2. * PI),
            //     (total_angle + error) / (2. * PI),
            // );
            log::info!("{}", state.velocity())
        }
    }
}
