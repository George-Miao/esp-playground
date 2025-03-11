use core::{
    f32::consts::PI,
    fmt::{Debug, Display},
};

use embedded_hal::i2c::I2c;
use esp_hal::time::{Duration, Instant};

use crate::util::Velocity;

const TWO_PI: f32 = 2. * PI;

pub trait SensorHardware {
    type Error: Debug;

    /// Reads the current angle in rad, results should range between 0 and 2π
    fn read_angle(&mut self) -> Result<f32, Self::Error>;
}

impl<I: I2c<Error: Debug>> SensorHardware for as5600::As5600<I> {
    type Error = as5600::error::Error<I::Error>;

    fn read_angle(&mut self) -> Result<f32, Self::Error> {
        Ok(self.raw_angle()? as f32 * TWO_PI / 4096.)
    }
}

/// A wrapper around sensor hardware that provides state
///
/// The state includes the current angle, the total angle, the number of full
/// rotations, the angular velocity, and the time since the last record. For
/// more, see [`SensorState`].
#[derive(Debug, PartialEq)]
pub struct Sensor<H> {
    inner: H,
    state: SensorState,
}

impl<I> Sensor<I> {
    pub fn new(hardware: I) -> Self {
        Self {
            inner: hardware,
            state: SensorState::default(),
        }
    }
}

impl<H: SensorHardware> Sensor<H> {
    pub fn update(&mut self) -> Result<(), H::Error> {
        self.state.record(self.inner.read_angle()?);

        Ok(())
    }

    pub fn reset(&mut self) {
        self.state = SensorState::default();
    }

    pub fn state(&self) -> SensorState {
        self.state
    }

    pub fn into_inner(self) -> H {
        self.inner
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorState {
    angle: f32,
    prev: Snapshot,
    full_rotations: i32,
    velocity: Velocity,
}

impl Default for SensorState {
    fn default() -> Self {
        Self {
            angle: 0.,
            prev: Snapshot {
                instant: Instant::now(),
                dt: Duration::from_millis(1),
                total_angle: 0.,
            },
            full_rotations: 0,
            velocity: Velocity::ZERO,
        }
    }
}

impl SensorState {
    fn record(&mut self, new_angle: f32) {
        let now = Instant::now();
        let angle_delta = new_angle - self.angle;

        // Difference between 2 angles is more than 4/5 revolution, consider it a full
        // rotation
        if angle_delta.abs() > 1.6 * PI {
            self.full_rotations -= angle_delta.signum() as i32;
        }

        self.angle = new_angle;
        let total_angle = self.total_angle();
        self.velocity = Velocity::rad(total_angle - self.prev.total_angle).per(self.prev.dt);
        self.prev = Snapshot {
            dt: now - self.prev.instant,
            instant: now,
            total_angle,
        };
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            instant: self.prev.instant,
            dt: self.prev.dt,
            total_angle: self.total_angle(),
        }
    }

    /// Current angle in rad
    pub fn angle(&self) -> f32 {
        self.angle
    }

    /// Total angle in rad
    pub fn total_angle(&self) -> f32 {
        self.full_rotations as f32 * TWO_PI + self.angle
    }

    /// Full rotations counter
    pub fn full_rotations(&self) -> i32 {
        self.full_rotations
    }

    /// Angular velocity in rad/s
    pub fn velocity(&self) -> Velocity {
        self.velocity
    }

    /// Duration between last 2 records
    pub fn last_dt(&self) -> Duration {
        self.prev.dt
    }
}

impl Display for SensorState {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{:.2} ({}, total {:.2}) @ {}",
            self.angle,
            self.full_rotations,
            self.total_angle(),
            self.velocity,
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Snapshot {
    instant: Instant,
    dt: Duration,
    total_angle: f32,
}

impl Snapshot {
    pub fn dt_secs(&self) -> f32 {
        self.dt.as_millis() as f32 * 1e-6
    }
}
