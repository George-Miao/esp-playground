use core::{
    f32::consts::{PI, SQRT_3},
    ops::{Deref, DerefMut},
};

use embedded_hal::pwm::SetDutyCycle;
use num_traits::float::FloatCore;
use tap::Pipe;

use crate::{
    RPM_TO_RADS, f,
    motor::BLDC,
    pid::{PIDController, VelocityPID},
    sensor::SensorHardware,
    util::Velocity,
};

pub struct Foc<M> {
    motor: M,
    motion_control: MotionControl,
    velocity_pid: VelocityPID,
    angle_pid: PIDController,
}

pub enum MotionControl {
    /// Target velocity in rad/μs
    Velocity(Velocity),

    /// Target angle in rad
    Angle(f32),

    /// Target torque
    Torque(f32),

    /// Number of ratchet steps
    Ratchet(RatchetState),

    /// Limit the position
    LimitPos(f32, f32),
}

pub struct RatchetState {
    steps: u8,
    rad_per_step: f32,
}

impl<M> Foc<M> {
    pub(crate) fn new(motor: M) -> Self {
        Self {
            motor,
            motion_control: MotionControl::Velocity(Velocity::ZERO),
            velocity_pid: PIDController::new()
                .p(0.02)
                .i(3.)
                .ramp(1000.)
                .limit(12.)
                .pipe(VelocityPID::new),
            angle_pid: PIDController::new().p(10.).limit(10.),
        }
    }

    /// Set the target velocity
    pub fn to_velocity(mut self, target: Velocity) -> Self {
        self.motion_control = MotionControl::Velocity(target);
        self
    }

    pub fn to_angle(mut self, target: f32) -> Self {
        self.motion_control = MotionControl::Angle(target);
        self
    }

    pub fn to_torque(mut self, target: f32) -> Self {
        self.motion_control = MotionControl::Torque(target);
        self
    }

    pub fn to_ratchet(mut self, num_step: u8) -> Self {
        self.motion_control = MotionControl::Ratchet(RatchetState {
            steps: num_step,
            rad_per_step: 2. * PI / num_step as f32,
        });
        self.angle_pid = self.angle_pid.p(240.).limit(240.);
        self.velocity_pid = self.velocity_pid.update(|pid| pid.p(0.03).i(0.));
        self
    }

    pub fn to_limit_pos(mut self, low: f32, high: f32) -> Self {
        assert!(low < high);
        self.motion_control = MotionControl::LimitPos(low, high);
        self
    }

    pub fn with_velocity_pid(mut self, controller: VelocityPID) -> Self {
        self.velocity_pid = controller;
        self
    }

    pub fn with_angle_pid(mut self, controller: PIDController) -> Self {
        self.angle_pid = controller;
        self
    }
}

impl<M> Deref for Foc<M> {
    type Target = M;

    fn deref(&self) -> &Self::Target {
        &self.motor
    }
}

impl<M> DerefMut for Foc<M> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.motor
    }
}

impl<H, A, B, C, const POLE: u8> Foc<BLDC<H, A, B, C, POLE>>
where
    H: SensorHardware,
    A: SetDutyCycle,
    B: SetDutyCycle<Error = A::Error>,
    C: SetDutyCycle<Error = A::Error>,
{
    fn calculate_qd(&self, target: Velocity) -> (f32, f32) {
        let state = self.motor.sensor.state();
        let voltage_limit = self.motor.voltage_limit;

        let target = target.as_secs();
        let current = state.velocity().as_secs();

        let voltage_bemf = self
            .motor
            .kv
            .map(|kv| current / (kv * SQRT_3) / RPM_TO_RADS)
            .unwrap_or_default();

        let q = self
            .phase_resistance
            .map(|r| target * r + voltage_bemf)
            .unwrap_or(target)
            .clamp(-voltage_limit, voltage_limit);
        let d = self
            .phase_inductance
            .map(|l| -target * current * POLE as f32 * l)
            .unwrap_or_default()
            .clamp(-voltage_limit, voltage_limit);

        // log::info!("target: {target:.2} q: {:.2}, d: {:.2}", q, d);

        (q, d)
    }

    pub fn tick(&mut self) -> Result<(), A::Error> {
        self.motor.sensor.update().expect("Failed to update sensor");

        let state = self.motor.sensor.state();
        let electrical_angle = self.motor.electrical_angle();
        let elapsed = state.last_dt();

        let velocity_limit = self.motor.velocity_limit;

        let (q, d) = match self.motion_control {
            MotionControl::LimitPos(low, high) => {
                todo!()
            }
            MotionControl::Torque(target) => self.calculate_qd(Velocity::per_sec(target)),
            MotionControl::Angle(target) => {
                if (target - state.total_angle()).abs() < 3e-2 {
                    return Ok(());
                }

                let velocity_target = self
                    .angle_pid
                    .compute(target, state.total_angle(), elapsed)
                    .pipe(Velocity::per_sec);

                let torque = self
                    .velocity_pid
                    .compute(velocity_target, state.velocity(), elapsed);

                self.calculate_qd(torque)
            }
            MotionControl::Velocity(target) => {
                let velocity = self.velocity_pid.compute(target, state.velocity(), elapsed);
                // log::info!("{} --({velocity})--> {target}", state.velocity());
                self.calculate_qd(velocity)
                // self.calculate_qd(target)
            }
            MotionControl::Ratchet(ref mut ratchet_state) => {
                let step = ratchet_state.rad_per_step;
                let total = state.total_angle();

                // Find the nearest step
                let target = (total / step).round() * step;
                if (target - total).abs() < 1e-2 {
                    return Ok(());
                }

                let velocity_target = self
                    .angle_pid
                    .compute(target, total, elapsed)
                    .pipe(Velocity::per_sec);

                let velocity = self
                    .velocity_pid
                    .compute(velocity_target, state.velocity(), elapsed)
                    .clamp(-velocity_limit, velocity_limit);

                self.calculate_qd(velocity)
            }
        };

        let v = self.motor.phase_voltage(f!(q), f!(d), electrical_angle);

        self.motor
            .pwm
            .set_voltage(v, f!(self.motor.voltage_power_supply))
    }
}
