use core::f32::consts::PI;

use embedded_hal::i2c::I2c;
use esp_hal::time::{now, Duration, Instant};

const TWO_PI: f32 = 2. * PI;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorDirection {
    Clockwise,
    CounterClockwise,
}

#[derive(Debug, PartialEq)]
pub struct As5600<I> {
    inner: as5600::As5600<I>,
    state: SensorState,
}

impl<I> As5600<I> {
    /// Counts per revolution
    pub const CPR: u16 = 4096;
}

impl<I: I2c> As5600<I> {
    pub fn new(i2c: I) -> Self {
        Self {
            inner: as5600::As5600::new(i2c),
            state: SensorState {
                angle: 0.,
                last_record: now(),
                last_dt: Duration::micros(1),
                full_rotations: 0,
                velocity: 0.,
                direction: SensorDirection::Clockwise,
            },
        }
    }

    pub fn update(&mut self) -> Result<(), as5600::error::Error<I::Error>> {
        let angle = self.inner.raw_angle()? as f32 * TWO_PI / Self::CPR as f32;
        self.state.record(angle);

        Ok(())
    }

    pub fn state(&self) -> SensorState {
        self.state
    }

    pub fn into_inner(self) -> as5600::As5600<I> {
        self.inner
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorState {
    angle: f32,
    last_record: Instant,
    last_dt: Duration,
    full_rotations: i32,
    velocity: f32,
    direction: SensorDirection,
}

impl SensorState {
    fn record(&mut self, new_angle: f32) {
        let now = now();
        let angle_delta = new_angle - self.angle;

        // Difference between 2 angles is more than 4/5 revolution, consider it a full rotation
        if angle_delta.abs() > 1.6 * PI {
            self.full_rotations += angle_delta.signum() as i32;
        }
        self.last_dt = now - self.last_record;
        self.velocity = angle_delta / self.last_dt.to_nanos() as f32 * 1e9;
        self.angle = new_angle;
        self.last_record = now;
    }

    /// Current angle in rad
    pub fn angle(&self) -> f32 {
        self.angle
    }

    /// Total angle in rad
    pub fn total_angle(&self) -> f32 {
        self.angle - self.full_rotations as f32 * TWO_PI
    }

    /// Full rotations counter
    pub fn full_rotations(&self) -> i32 {
        self.full_rotations
    }

    /// Angular velocity in rad/s
    pub fn velocity(&self) -> f32 {
        self.velocity
    }

    /// Duration between last 2 records
    pub fn last_dt(&self) -> Duration {
        self.last_dt
    }
}
