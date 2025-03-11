use core::{
    f32::consts::PI,
    fmt::{self, Display, Formatter},
    ops::{Add, Div, Mul, Neg, Sub},
};

use esp_hal::time::Duration;

/// Angular velocity in radians per second (rad/s)
#[derive(Debug, Clone, Copy, PartialOrd, PartialEq)]
pub struct Velocity(f32);

impl Velocity {
    /// 1 revolution per minute
    pub const RPM: Velocity = Velocity::per_sec(PI / 30.);
    /// 1 revolution (2π rad) per second
    pub const RPS: Velocity = Velocity::per_sec(2. * PI);
    pub const ZERO: Velocity = Velocity(0.0);

    pub const fn rad(radian: f32) -> VelocityBuilder {
        VelocityBuilder(radian)
    }

    pub const fn degree(degree: f32) -> VelocityBuilder {
        VelocityBuilder(degree.to_radians())
    }

    pub const fn as_secs(self) -> f32 {
        self.0
    }

    pub const fn as_millis(self) -> f32 {
        self.0 * 1e-3
    }

    pub const fn as_micros(self) -> f32 {
        self.0 * 1e-6
    }

    pub const fn per_sec(rad: f32) -> Self {
        Self(rad)
    }

    pub const fn per_milli(rad: f32) -> Self {
        Self(rad * 1e3)
    }

    pub const fn per_micro(rad: f32) -> Self {
        Self(rad * 1e6)
    }

    pub const fn clamp(self, min: Velocity, max: Velocity) -> Self {
        if self.0 < min.0 {
            min
        } else if self.0 > max.0 {
            max
        } else {
            self
        }
    }
}

impl Neg for Velocity {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

macro_rules! mul {
    ($($ty:ty),*) => {
        $(
            impl Mul<$ty> for Velocity {
                type Output = Self;

                fn mul(self, rhs: $ty) -> Self::Output {
                    Self(self.0 * rhs as f32)
                }
            }

            impl Mul<Velocity> for $ty {
                type Output = Velocity;

                fn mul(self, rhs: Velocity) -> Self::Output {
                    Velocity(self as f32 * rhs.0)
                }
            }
        )*
    };
}

mul!(f32, f64);
mul!(u8, u16, u32, u64, u128);
mul!(i8, i16, i32, i64, i128);

impl Div<f32> for Velocity {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        Self(self.0 / rhs)
    }
}

impl Add for Velocity {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Velocity {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Display for Velocity {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        write!(f, "{:.2}rad/s", self.as_secs())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityBuilder(f32);

impl VelocityBuilder {
    pub const fn per(self, duration: Duration) -> Velocity {
        self.per_micro(duration.as_micros() as f32)
    }

    pub const fn per_sec(self, second: f32) -> Velocity {
        Velocity(self.0 / second)
    }

    pub const fn per_milli(self, milli: f32) -> Velocity {
        Velocity(self.0 / milli * 1e3)
    }

    pub const fn per_micro(self, micros: f32) -> Velocity {
        Velocity(self.0 / micros * 1e6)
    }

    pub const fn per_nano(self, nanos: f32) -> Velocity {
        Velocity(self.0 / nanos * 1e9)
    }
}
