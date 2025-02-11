#![no_std]

pub mod motor;
pub mod sensor;

use fixed::types::I16F16;

const SQRT3_2: I16F16 = f!("0.86602540378");

macro_rules! f {
    ($x: literal) => {
        ::fixed::types::I16F16::lit($x)
    };
    ($x: expr) => {
        ::fixed::types::I16F16::from_num($x)
    };
}

pub(crate) use f;
