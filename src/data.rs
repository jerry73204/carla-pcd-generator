use carla::{geom::Location, sensor::data::SemanticLidarDetection};
use pcd_rs::PcdSerialize;

/// The point type that is written to the .pcd file.
#[derive(PcdSerialize)]
pub struct PcdPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub cos_inc_angle: f32,
    pub object_idx: u32,
    pub object_tag: u32,
}

impl From<&SemanticLidarDetection> for PcdPoint {
    fn from(from: &SemanticLidarDetection) -> Self {
        let SemanticLidarDetection {
            point: Location { x, y, z },
            cos_inc_angle,
            object_idx,
            object_tag,
        } = *from;

        Self {
            x,
            y,
            z,
            cos_inc_angle,
            object_idx,
            object_tag,
        }
    }
}
