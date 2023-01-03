use std::path::Path;

use carla::sensor::data::SemanticLidarMeasurement;

pub struct LidarMessage {
    pub frame_id: usize,
    pub role_name: String,
    pub measure: SemanticLidarMeasurement,
    pub sub_outdir: &'static Path,
}
