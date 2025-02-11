use crate::{f, sensor::As5600, SQRT3_2};
use cordic::sin_cos;
use core::f32::consts::PI;
use esp_hal::delay::Delay;
use fixed::types::I16F16;

use embedded_hal::{i2c::I2c, pwm::SetDutyCycle};

mod_use::mod_use![open_loop, foc];

pub struct BLDC<I, A, B, C, const POLE: u8> {
    sensor: As5600<I>,
    pwm: ThreePhasePwm<A, B, C>,
    zero_electrical_angle: Option<f32>,
    voltage_limit: f32,
}

impl<I, A, B, C> BLDC<I, A, B, C, 0>
where
    I: I2c,
    A: SetDutyCycle,
    B: SetDutyCycle<Error = A::Error>,
    C: SetDutyCycle<Error = A::Error>,
{
    pub fn new<const POLE: u8>(
        i2c: I,
        pwm: ThreePhasePwm<A, B, C>,
        voltage_limit: f32,
    ) -> BLDC<I, A, B, C, POLE> {
        BLDC {
            sensor: As5600::new(i2c),
            zero_electrical_angle: None,
            pwm,
            voltage_limit,
        }
    }
}

impl<I, A, B, C, const POLE: u8> BLDC<I, A, B, C, POLE>
where
    I: I2c,
    A: SetDutyCycle,
    B: SetDutyCycle<Error = A::Error>,
    C: SetDutyCycle<Error = A::Error>,
{
    pub fn sensor(&self) -> &As5600<I> {
        &self.sensor
    }

    pub fn sensor_mut(&mut self) -> &mut As5600<I> {
        &mut self.sensor
    }

    pub fn open_loop(self, velocity: f32) -> OpenLoop<Self> {
        OpenLoop::new(self, velocity)
    }

    /// FOC with velocity control and a single velocity PI controller
    pub fn foc(self, velocity: f32) -> Foc<Self> {
        Foc::new(self, velocity)
    }

    pub fn aligned(mut self) -> Self {
        self.align();
        self
    }

    pub fn align(&mut self) {
        let vol = self.phase_voltage(f!(self.voltage_limit) / 2, I16F16::ZERO, 1.5 * PI);
        self.pwm.set_voltage(vol, f!(self.voltage_limit)).unwrap();
        Delay::new().delay_millis(700);
        self.zero_electrical_angle = None;
        self.zero_electrical_angle = Some(self.electrical_angle());
    }

    /// Read the electrical angle
    pub(crate) fn electrical_angle(&mut self) -> f32 {
        normalize_angle(
            self.mechanical_angle() * POLE as f32 - self.zero_electrical_angle.unwrap_or_default(),
        )
    }

    /// Read the mechanical angle
    pub(crate) fn mechanical_angle(&mut self) -> f32 {
        self.sensor.update().unwrap();
        self.sensor.state().angle()
    }

    pub(crate) fn phase_voltage(
        &self,
        volt_q: I16F16,
        volt_d: I16F16,
        target_angle: f32,
    ) -> (I16F16, I16F16, I16F16) {
        let (sin, cos) = sin_cos(f!(target_angle));
        let alpha = cos * volt_d - sin * volt_q;
        let beta = sin * volt_d + cos * volt_q;

        let mut a = alpha;
        let mut b = f!("-0.5") * alpha + SQRT3_2 * beta;
        let mut c = f!("-0.5") * alpha - SQRT3_2 * beta;

        // Space vector modulation
        let min = a.min(b).min(c);
        let max = a.max(b).max(c);

        let center = f!(self.voltage_limit) / 2 - (max + min) / 2;

        a += center;
        b += center;
        c += center;

        (a, b, c)
    }
}

pub struct ThreePhasePwm<A, B, C> {
    pub a: A,
    pub b: B,
    pub c: C,
}

impl<A, B, C> ThreePhasePwm<A, B, C>
where
    A: SetDutyCycle,
    B: SetDutyCycle<Error = A::Error>,
    C: SetDutyCycle<Error = A::Error>,
{
    fn volt_to_percent(volt: I16F16, volt_max: I16F16) -> u16 {
        (volt * 100 / volt_max)
            .clamp(I16F16::ZERO, I16F16::lit("100"))
            .to_num()
    }

    pub fn set_voltage(
        &mut self,
        duty: (I16F16, I16F16, I16F16),
        volt_max: I16F16,
    ) -> Result<(), A::Error> {
        self.set_duty((
            Self::volt_to_percent(duty.0, volt_max),
            Self::volt_to_percent(duty.1, volt_max),
            Self::volt_to_percent(duty.2, volt_max),
        ))
    }

    pub fn set_duty(&mut self, duty: (u16, u16, u16)) -> Result<(), A::Error> {
        // info!("{} {} {}", duty.0, duty.1, duty.2);

        self.a.set_duty_cycle(duty.0)?;
        self.b.set_duty_cycle(duty.1)?;
        self.c.set_duty_cycle(duty.2)?;

        Ok(())
    }
}

fn normalize_angle(angle: f32) -> f32 {
    let a = angle % (2. * PI);
    if a < 0. {
        a + 2. * PI
    } else {
        a
    }
}
