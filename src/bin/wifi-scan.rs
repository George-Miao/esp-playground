#![feature(cell_update, asm_experimental_arch)]
#![no_std]
#![no_main]

extern crate alloc;

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock, delay::Delay, rng::Rng, timer::timg::TimerGroup, xtensa_lx_rt::entry,
};
use fugit::ExtU64;
use log::info;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);
    let peripherals: esp_hal::peripherals::Peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let rng = Rng::new(peripherals.RNG);
    let ctrl = esp_wifi::init(timg0.timer0, rng, peripherals.RADIO_CLK).unwrap();

    let mut wifi = peripherals.WIFI;

    let (_, mut controller) = esp_wifi::wifi::new(&ctrl, &mut wifi).unwrap();
    controller
        .set_power_saving(esp_wifi::config::PowerSaveMode::None)
        .unwrap();
    controller.start().unwrap();
    let delay = Delay::new();

    loop {
        let (res, n) = controller.scan_n::<100>().unwrap();
        info!("Found {n} SSID(s)");
        for res in res {
            info!("{res:?}");
        }
        delay.delay(500.millis());
    }
}
