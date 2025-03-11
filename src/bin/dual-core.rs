#![feature(cell_update)]
#![no_std]
#![no_main]

use alloc::boxed::Box;
use core::cell::Cell;

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    system::{CpuControl, Stack},
    time::Duration,
    xtensa_lx_rt::entry,
};
use fugit::ExtU64;
use log::info;

extern crate alloc;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);

    let stack = Box::new(Stack::<8096>::new());
    let stack = Box::leak(stack);
    let peripherals: esp_hal::peripherals::Peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let counter = critical_section::Mutex::new(Cell::new(0i32));
    let mut ctrl = CpuControl::new(peripherals.CPU_CTRL);

    let _g = ctrl.start_app_core(stack, || cpu1_task(&Delay::new(), &counter));
    let delay = Delay::new();
    loop {
        critical_section::with(|cs| {
            let val = counter.borrow(cs).update(|x| x + 1);
            info!("C0: {}", val);
        });
        delay.delay(Duration::from_millis(500));
    }
}

fn cpu1_task(delay: &Delay, counter: &critical_section::Mutex<Cell<i32>>) -> ! {
    loop {
        critical_section::with(|cs| {
            let val = counter.borrow(cs).update(|x| x + 10);
            info!("C1: {}", val);
        });
        delay.delay(Duration::from_millis(1200));
    }
}
