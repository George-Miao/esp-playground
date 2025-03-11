use embedded_hal::pwm::SetDutyCycle;
use esp_hal::time::Instant;
use fixed::types::I16F16;

use crate::{f, motor::BLDC};

pub struct OpenLoop<M> {
    motor: M,
    prev_tick: Instant,
    shaft_angle: f32,
    velocity: f32,
}

impl<M> OpenLoop<M> {
    pub fn new(motor: M, velocity: f32) -> Self {
        Self {
            motor,
            prev_tick: Instant::now(),
            shaft_angle: 0.,
            velocity,
        }
    }

    pub fn velocity(&self) -> f32 {
        self.velocity
    }

    pub fn set_velocity(&mut self, velocity: f32) {
        self.velocity = velocity;
    }

    pub fn update_velocity(&mut self, delta: f32) {
        self.velocity += delta;
    }

    pub fn into_inner(self) -> M {
        self.motor
    }
}

impl<H, A, B, C, const POLE: u8> OpenLoop<BLDC<H, A, B, C, POLE>>
where
    A: SetDutyCycle,
    B: SetDutyCycle<Error = A::Error>,
    C: SetDutyCycle<Error = A::Error>,
{
    pub fn tick(&mut self) -> Result<(), A::Error> {
        let now = Instant::now();
        let dt = (now - self.prev_tick).as_micros() as f32 * 1e-6; // Delta in seconds
        let limit = f!(self.motor.voltage_limit);

        // self.shaft_angle = normalize_angle(self.shaft_angle + dt * self.velocity);
        self.shaft_angle += dt * self.velocity;

        let electronic_angle = self.shaft_angle * POLE as f32;

        let phase_voltage = self
            .motor
            .phase_voltage(limit / 2, I16F16::ZERO, electronic_angle);
        self.motor.pwm.set_voltage(phase_voltage, limit)?;
        self.prev_tick = now;

        Ok(())
    }
}
