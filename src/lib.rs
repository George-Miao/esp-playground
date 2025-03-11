#![feature(
    cell_update,
    type_changing_struct_update,
    more_float_constants,
    const_float_methods,
    never_type
)]
#![no_std]

pub mod display;
pub mod dma;
pub mod motor;
mod pid;
pub mod sensor;
pub mod util;

use core::cell::Cell;

use critical_section::Mutex;
use fixed::types::I16F16;

static CAN_LOG: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

pub fn can_log() -> bool {
    critical_section::with(|cs| CAN_LOG.borrow(cs).get())
}

pub fn set_can_log(can_log: bool) {
    critical_section::with(|cs| CAN_LOG.borrow(cs).set(can_log));
}

const SQRT3_2: I16F16 = f!("0.86602540378");
const RPM_TO_RADS: f32 = 0.104_719_76;

macro_rules! f {
    ($x:literal) => {
        ::fixed::types::I16F16::lit($x)
    };
    ($x:expr) => {
        ::fixed::types::I16F16::from_num($x)
    };
}

pub(crate) use f;
