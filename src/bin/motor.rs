#![no_std]
#![no_main]

extern crate alloc;

use core::f32::consts::PI;

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    i2c::{self, master::I2c},
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, McPwm},
};
use esp_hal::{gpio::Output, time::RateExtU32};
use esp_hal::{mcpwm::PeripheralClockConfig, xtensa_lx_rt::entry};

use playground::motor::{ThreePhasePwm, BLDC};

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);

    let peripherals: esp_hal::peripherals::Peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let _en = Output::new(peripherals.GPIO3, esp_hal::gpio::Level::High);
    let in1 = peripherals.GPIO10;
    let in2 = peripherals.GPIO9;
    let in3 = peripherals.GPIO46;

    let clock_cfg = PeripheralClockConfig::with_frequency(32.MHz()).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);

    let a = mcpwm
        .operator0
        .with_pin_a(in1, PwmPinConfig::UP_ACTIVE_HIGH);

    let b = mcpwm
        .operator1
        .with_pin_a(in2, PwmPinConfig::UP_ACTIVE_HIGH);

    let c = mcpwm
        .operator2
        .with_pin_a(in3, PwmPinConfig::UP_ACTIVE_HIGH);

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 20.kHz())
        .unwrap();

    mcpwm.timer0.start(timer_clock_cfg);

    let i2c = I2c::new(peripherals.I2C0, i2c::master::Config::default())
        .unwrap()
        .with_scl(peripherals.GPIO18)
        .with_sda(peripherals.GPIO17);

    let mut drive = BLDC::new::<7>(i2c, ThreePhasePwm { a, b, c }, 8.)
        .aligned()
        .foc(4. * PI);

    loop {
        drive.tick();
    }
}
