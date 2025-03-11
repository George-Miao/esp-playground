use core::{
    cell::{RefCell, UnsafeCell},
    fmt::{self, Formatter},
};

use critical_section::{Mutex, with};
use esp_hal::time::{Duration, Instant};

enum SamplingState {
    Disabled { prev_time: Instant },
    Enabled(SamplingPoint),
}

static SAMPLING: Mutex<RefCell<SamplingState>> =
    Mutex::new(RefCell::new(SamplingState::Disabled {
        prev_time: Instant::EPOCH,
    }));

#[derive(Clone, Copy)]
pub struct SamplingPoint {
    pub now: Instant,
    pub dt: Duration,
}

pub fn sample<F>(f: F)
where
    F: for<'a> FnMut(&mut Formatter<'a>, SamplingPoint) -> fmt::Result,
{
    struct Shim<F> {
        f: UnsafeCell<F>,
        point: SamplingPoint,
    }

    impl<F> fmt::Display for Shim<F>
    where
        F: for<'a> FnMut(&mut Formatter<'a>, SamplingPoint) -> fmt::Result,
    {
        fn fmt(&self, f: &mut Formatter) -> fmt::Result {
            (unsafe { &mut *self.f.get() })(f, self.point)
        }
    }

    critical_section::with(|cs| match *SAMPLING.borrow(cs).borrow() {
        SamplingState::Disabled { .. } => {}
        SamplingState::Enabled(point) => {
            let shim = Shim {
                f: UnsafeCell::new(f),
                point,
            };

            log::info!("[S] {shim}");
        }
    });
}

pub struct Guard {
    enabled: bool,
    prev_time: Instant,
}

impl Drop for Guard {
    fn drop(&mut self) {
        if self.enabled {
            with(|cs| {
                SAMPLING.borrow(cs).replace(SamplingState::Disabled {
                    prev_time: self.prev_time,
                })
            });
        }
    }
}

pub fn enable_sampling(enable: bool) -> Guard {
    let prev_time = if enable {
        let now = Instant::now();
        with(|cs| {
            SAMPLING.borrow(cs).replace_with(|prev| {
                let prev_time = match prev {
                    SamplingState::Disabled { prev_time } => *prev_time,
                    SamplingState::Enabled(_) => panic!("Sampling already enabled"),
                };

                SamplingState::Enabled(SamplingPoint {
                    now,
                    dt: now - prev_time,
                })
            });
        });
        now
    } else {
        Instant::EPOCH
    };

    Guard {
        enabled: enable,
        prev_time,
    }
}
