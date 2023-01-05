use crate::config::{self, Config};
use std::num::NonZeroUsize;

pub const DEFAULT_CONFIG: Config = Config {
    lidar: config::Lidar {
        range_m: 40.0,
        height_m: 1.6,
        n_channels: unsafe { NonZeroUsize::new_unchecked(32) },
        rotation_freq: unsafe { NonZeroUsize::new_unchecked(10) },
    },
};
