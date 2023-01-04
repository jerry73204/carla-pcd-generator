/// The mod includes hacks to convert types of nalgebra 0.31 to types of nalgebra 0.30.

pub fn convert_isometry3(orig: &nalgebra::Isometry3<f32>) -> kiss3d::nalgebra::Isometry3<f32> {
    kiss3d::nalgebra::Isometry3 {
        rotation: convert_rotation(&orig.rotation),
        translation: convert_translation(&orig.translation),
    }
}

pub fn convert_translation(
    orig: &nalgebra::Translation3<f32>,
) -> kiss3d::nalgebra::Translation3<f32> {
    kiss3d::nalgebra::Translation3 {
        vector: kiss3d::nalgebra::Vector3::new(orig.x, orig.y, orig.z),
    }
}

pub fn convert_rotation(
    orig: &nalgebra::UnitQuaternion<f32>,
) -> kiss3d::nalgebra::UnitQuaternion<f32> {
    let vec = orig.quaternion().coords;
    kiss3d::nalgebra::UnitQuaternion::from_quaternion(kiss3d::nalgebra::Quaternion {
        coords: kiss3d::nalgebra::Vector4::new(vec.x, vec.y, vec.z, vec.w),
    })
}
