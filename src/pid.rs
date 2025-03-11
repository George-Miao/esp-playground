use core::f32;

use esp_hal::time::Duration;
use tap::Pipe;

use crate::util::Velocity;

#[derive(Clone, Copy, Debug)]
pub struct PIDController {
    p: f32,
    i: f32,
    d: f32,

    /// Rate limit for output
    output_ramp: Option<f32>,

    /// Limit of output and integral
    limit: f32,

    /// State
    state: PIDState,
}

#[derive(Clone, Copy, Debug)]
struct PIDState {
    /// Integral
    integral: f32,

    /// Previous error
    err: f32,

    /// Previous output
    output: f32,
}

impl PIDController {
    pub fn new() -> Self {
        Self {
            p: 0.0,
            i: 0.0,
            d: 0.0,
            output_ramp: None,
            limit: f32::MAX,
            state: PIDState {
                integral: 0.0,
                err: 0.0,
                output: 0.0,
            },
        }
    }

    pub fn p(mut self, p: f32) -> Self {
        self.p = p;
        self
    }

    pub fn i(mut self, i: f32) -> Self {
        self.i = i;
        self
    }

    pub fn d(mut self, d: f32) -> Self {
        self.d = d;
        self
    }

    pub fn ramp(mut self, ramp: f32) -> Self {
        self.output_ramp = Some(ramp);
        self
    }

    pub fn limit(mut self, limit: f32) -> Self {
        self.limit = limit;
        self
    }

    pub fn compute(&mut self, target: f32, measure: f32, dt: Duration) -> f32 {
        let dt = dt.as_micros() as f32 * 1e-6;

        let err = target - measure;

        let p = self.p * err;
        let i = (self.state.integral + self.i * dt * 0.5 * (err + self.state.err))
            .clamp(-self.limit, self.limit);
        let d = self.d * (err - self.state.err) / dt;

        let mut output = (p + i + d).clamp(-self.limit, self.limit);

        if let Some(ramp) = self.output_ramp {
            let rate = (output - self.state.output) / dt;
            if rate > ramp {
                output = self.state.output + ramp * dt;
            } else if rate < -ramp {
                output = self.state.output - ramp * dt;
            }
        }

        // log::info!(
        //     "{target:.2} - {measure:.2} = {err:.2} ==> p = {p:.2} i = {:.2} d =
        // {d:.2}",     i
        // );

        self.state.integral = i;
        self.state.err = err;
        self.state.output = output;

        output
    }
}

impl Default for PIDController {
    fn default() -> Self {
        Self::new()
    }
}

pub struct VelocityPID(PIDController);

impl VelocityPID {
    pub fn update<F: FnOnce(PIDController) -> PIDController>(self, f: F) -> Self {
        Self(f(self.0))
    }

    pub fn new(inner: PIDController) -> Self {
        Self(inner)
    }

    pub fn compute(&mut self, target: Velocity, measure: Velocity, dt: Duration) -> Velocity {
        self.0
            .compute(target.as_secs(), measure.as_secs(), dt)
            .pipe(Velocity::per_sec)
    }
}
