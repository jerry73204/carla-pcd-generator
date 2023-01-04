use carla::sensor::data::SemanticLidarMeasurement;
use nalgebra::Isometry3;
use std::{path::Path, sync::Arc};

/// The message is published by Lidar sensors in listen().
pub struct LidarMessage {
    pub frame_id: usize,
    pub role_name: &'static str,
    pub measure: Arc<SemanticLidarMeasurement>,
    pub sub_outdir: &'static Path,
}

/// The message is sent to GUI component.
pub struct GuiMessage {
    pub role_name: &'static str,
    pub measure: Arc<SemanticLidarMeasurement>,
    pub transform: Isometry3<f32>,
}
