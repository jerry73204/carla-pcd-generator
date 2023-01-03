use carla::sensor::data::SemanticLidarMeasurement;
use std::path::Path;

/// The message is published by Lidar sensors in listen().
pub struct LidarMessage {
    pub frame_id: usize,
    pub role_name: String,
    pub measure: SemanticLidarMeasurement,
    pub sub_outdir: &'static Path,
}
