use core::f32::consts::PI;

use cordic::sin_cos;
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::delay::Delay;
use fixed::types::I16F16;

use crate::{
    SQRT3_2, f,
    sensor::{Sensor, SensorHardware},
    util::Velocity,
};

mod_use::mod_use![open_loop, foc];

const DEFAULT_VOLTAGE_SUPPLY: f32 = 12.;

pub struct BLDC<H, A, B, C, const POLE: u8> {
    sensor: Sensor<H>,
    pwm: ThreePhasePwm<A, B, C>,
    zero_electrical_angle: Option<f32>,
    voltage_limit: f32,
    velocity_limit: Velocity,
    voltage_power_supply: f32,
    kv: Option<f32>,
    phase_resistance: Option<f32>,
    phase_inductance: Option<f32>,
}

impl<A, B, C> BLDC<(), A, B, C, 0> {
    pub fn new<const POLE: u8>(pwm: ThreePhasePwm<A, B, C>) -> BLDC<(), A, B, C, POLE> {
        BLDC {
            sensor: Sensor::new(()),
            zero_electrical_angle: None,
            pwm,
            voltage_limit: 12.,
            velocity_limit: Velocity::rad(60. * PI).per_sec(1.), // 30 RPS or 1800 RPM
            voltage_power_supply: DEFAULT_VOLTAGE_SUPPLY,
            kv: None,
            phase_resistance: None,
            phase_inductance: None,
        }
    }
}

impl<H, A, B, C, const POLE: u8> BLDC<H, A, B, C, POLE> {
    pub fn with_voltage_limit(mut self, limit: f32) -> Self {
        self.voltage_limit = limit;
        self
    }

    pub fn with_voltage_power_supply(mut self, supply: f32) -> Self {
        self.voltage_power_supply = supply;
        self
    }

    pub fn with_kv(mut self, kv: f32) -> Self {
        self.kv = Some(kv);
        self
    }

    pub fn with_phase_resistance(mut self, resistance: f32) -> Self {
        self.phase_resistance = Some(resistance);
        self
    }

    pub fn with_phase_inductance(mut self, inductance: f32) -> Self {
        self.phase_inductance = Some(inductance);
        self
    }

    pub fn with_sensor<N>(self, sensor: N) -> BLDC<N, A, B, C, POLE> {
        BLDC {
            sensor: Sensor::new(sensor),
            ..self
        }
    }

    pub fn sensor(&self) -> &Sensor<H> {
        &self.sensor
    }

    pub fn sensor_mut(&mut self) -> &mut Sensor<H> {
        &mut self.sensor
    }
}

impl<H, A, B, C, const POLE: u8> BLDC<H, A, B, C, POLE>
where
    H: SensorHardware,
{
    /// Read the electrical angle
    ///
    /// To get up-to-date angle, call [`Sensor::update`] before calling this
    /// method.
    pub(crate) fn electrical_angle(&self) -> f32 {
        normalize_angle(
            self.sensor.state().angle() * POLE as f32
                - self.zero_electrical_angle.unwrap_or_default(),
        )
    }
}

impl<H, A, B, C, const POLE: u8> BLDC<H, A, B, C, POLE>
where
    H: SensorHardware,
    A: SetDutyCycle,
    B: SetDutyCycle<Error = A::Error>,
    C: SetDutyCycle<Error = A::Error>,
{
    /// FOC with velocity control and a single velocity PI controller
    pub fn foc(self) -> Foc<Self> {
        Foc::new(self)
    }

    pub fn align(&mut self) -> Result<(), H::Error> {
        self.sensor.update()?;
        let angle = self.electrical_angle();
        let vol = self.phase_voltage(f!(self.voltage_power_supply) / 2, I16F16::ZERO, 1.5 * PI);
        self.pwm
            .set_voltage(vol, f!(self.voltage_power_supply))
            .unwrap();
        Delay::new().delay_millis(700);
        self.zero_electrical_angle = None;
        self.sensor.update()?;

        // Motor has moved a little bit, confirm alignment
        if self.electrical_angle() != angle {
            self.zero_electrical_angle = Some(self.electrical_angle());
        } else {
            panic!("Failed to align motor");
        }

        self.sensor.reset();

        Ok(())
    }

    pub fn aligned(mut self) -> Result<Self, H::Error> {
        self.align()?;
        Ok(self)
    }
}

impl<H, A, B, C, const POLE: u8> BLDC<H, A, B, C, POLE>
where
    A: SetDutyCycle,
    B: SetDutyCycle<Error = A::Error>,
    C: SetDutyCycle<Error = A::Error>,
{
    pub fn open_loop(self, velocity: f32) -> OpenLoop<Self> {
        OpenLoop::new(self, velocity)
    }

    pub(crate) fn phase_voltage(
        &self,
        volt_q: I16F16,
        volt_d: I16F16,
        angle: f32,
    ) -> (I16F16, I16F16, I16F16) {
        let (sin, cos) = sin_cos(f!(angle));
        let alpha = cos * volt_d - sin * volt_q;
        let beta = sin * volt_d + cos * volt_q;

        let mut a = alpha;
        let mut b = f!("-0.5") * alpha + SQRT3_2 * beta;
        let mut c = f!("-0.5") * alpha - SQRT3_2 * beta;

        // Space vector modulation
        let min = a.min(b).min(c);
        let max = a.max(b).max(c);

        let center = f!(self.voltage_power_supply) / 2 - (max + min) / 2;

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
        self.a.set_duty_cycle(duty.0)?;
        self.b.set_duty_cycle(duty.1)?;
        self.c.set_duty_cycle(duty.2)?;

        Ok(())
    }
}

fn normalize_angle(angle: f32) -> f32 {
    let a = angle % (2. * PI);
    if a < 0. { a + 2. * PI } else { a }
}
