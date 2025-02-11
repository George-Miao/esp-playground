use embedded_hal::{i2c::I2c, pwm::SetDutyCycle};
use fixed::types::I16F16;
use foc::pid::PIController;

use crate::{f, motor::BLDC};

pub struct Foc<M> {
    motor: M,
    velocity_pi: PIController,
    target_velocity: I16F16,
}

impl<M> Foc<M> {
    pub fn new(motor: M, velocity: f32) -> Self {
        Self {
            motor,
            velocity_pi: PIController::new(f!("0.0002"), f!("0.001")),
            target_velocity: f!(velocity * 1e-6), // Convert to rad/μs
        }
    }
}

impl<I, A, B, C, const POLE: u8> Foc<BLDC<I, A, B, C, POLE>>
where
    I: I2c,
    A: SetDutyCycle,
    B: SetDutyCycle<Error = A::Error>,
    C: SetDutyCycle<Error = A::Error>,
{
    pub fn tick(&mut self) {
        self.motor.sensor.update().unwrap();
        let state = self.motor.sensor.state();
        let set_point = self.velocity_pi.update(
            f!(state.velocity() * 1e-6),
            self.target_velocity,
            f!(state.last_dt().to_micros()),
        );
        let electrical_angle = self.motor.electrical_angle();
        let (a, b, c) = self
            .motor
            .phase_voltage(set_point, I16F16::ZERO, electrical_angle);

        self.motor
            .pwm
            .set_voltage((a, b, c), f!(self.motor.voltage_limit))
            .unwrap();

        // log::info!("{electrical_angle:.2} | {state:?}");
    }
}
