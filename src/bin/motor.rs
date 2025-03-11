#![no_std]
#![no_main]

extern crate alloc;

use core::f32::consts::PI;

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, Output, Pull},
    i2c::{self, master::I2c},
    mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode},
    time::{Instant, Rate},
    xtensa_lx_rt::entry,
};
use log::info;
use playground::{
    motor::{BLDC, ThreePhasePwm},
    util::Velocity,
};
use tap::Pipe;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);
    let peripherals: esp_hal::peripherals::Peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    // let button = Input::new(peripherals.GPIO7, Pull::None);
    let mut en = Output::new(
        peripherals.GPIO4,
        esp_hal::gpio::Level::High,
        Default::default(),
    );
    let in1 = peripherals.GPIO7;
    let in2 = peripherals.GPIO6;
    let in3 = peripherals.GPIO5;

    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(16)).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);

    let mut a = mcpwm
        .operator0
        .with_pin_a(in1, PwmPinConfig::UP_ACTIVE_HIGH);

    let mut b = mcpwm
        .operator1
        .with_pin_a(in2, PwmPinConfig::UP_ACTIVE_HIGH);

    let mut c = mcpwm
        .operator2
        .with_pin_a(in3, PwmPinConfig::UP_ACTIVE_HIGH);

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, Rate::from_khz(20))
        .unwrap();

    mcpwm.timer0.start(timer_clock_cfg);

    // a.set_timestamp(100);
    // b.set_timestamp(100);
    // c.set_timestamp(100);

    // loop {}

    let encoder = I2c::new(peripherals.I2C0, i2c::master::Config::default())
        .unwrap()
        .with_scl(peripherals.GPIO12)
        .with_sda(peripherals.GPIO11)
        .pipe(as5600::As5600::new);

    let mut drive = BLDC::new::</* Pole Pair Number */ 7>(ThreePhasePwm { a, b, c })
        .with_voltage_power_supply(12.)
        .with_sensor(encoder)
        // .with_phase_inductance(0.86 * 1e-3) // 0.86mH
        // .with_phase_resistance(2.3) // 2.3 Ohm
        // .with_kv(220.) // 220 RPM/V
        .aligned()
        .unwrap()
        .foc()
        // .open_loop(PI * 2.);
        // .to_torque(PI);
        // .to_angle(0.);
        .to_ratchet(5);
    // .to_velocity(10 * Velocity::RPS);

    let mut last_sampling = (0., Instant::now());
    let mut tick = 0;
    let mut button_cooldown_start = Instant::EPOCH;

    loop {
        // if button.is_high() && (now() - button_cooldown_start).to_millis() > 200 {
        //     log::info!("Button pressed, toggle motor");
        //     en.toggle();
        //     button_cooldown_start = now();
        // }

        // if en.is_set_low() {
        //     continue;
        // }

        // if tick % 1000 == 0 {
        //     let (last, time) = last_sampling;
        //     let now = now();
        //     let curr = drive.sensor().state().total_angle();
        //     let vel = Velocity::rad(curr - last).per(now - time);
        //     log::info!("{vel}");
        //     last_sampling = (curr, now);
        // }

        tick += 1;
        drive.tick().unwrap()
    }
}
