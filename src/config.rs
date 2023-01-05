use serde::Deserialize;
use std::num::NonZeroUsize;

#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub lidar: Lidar,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Lidar {
    pub range_m: f32,
    pub height_m: f32,
    pub n_channels: NonZeroUsize,
    pub rotation_freq: NonZeroUsize,
}
