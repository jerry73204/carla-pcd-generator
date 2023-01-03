use std::path::{Path, PathBuf};

use crate::message::LidarMessage;
use anyhow::Result;
use carla::{
    client::{Sensor, Vehicle, World},
    sensor::data::SemanticLidarMeasurement,
};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use tracing::warn;

pub struct VehicleAgent {
    pub role_name: String,
    _vehicle: Vehicle,
    _lidar: Sensor,
}

impl VehicleAgent {
    pub fn new(
        world: &mut World,
        role_name: &str,
        spawn_point: &Isometry3<f32>,
        lidar_tx: flume::Sender<LidarMessage>,
        sub_outdir: PathBuf,
    ) -> Result<Self> {
        let role_name = role_name.to_string();
        let sub_outdir: &'static Path = Box::leak(Box::new(sub_outdir));

        let vehicle = world
            .actor_builder("vehicle.tesla.model3")?
            .set_attribute("role_name", &role_name)?
            .spawn_vehicle(spawn_point)?;
        vehicle.set_autopilot(true);

        let lidar_pose = Isometry3 {
            rotation: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            translation: Translation3::new(0.0, 0.0, 1.6),
        };
        let lidar = world
            .actor_builder("sensor.lidar.ray_cast_semantic")?
            .set_attribute("channels", "32")?
            .set_attribute("points_per_second", "90000")?
            .set_attribute("rotation_frequency", "40")?
            .set_attribute("range", "20")?
            .spawn_sensor_opt(&lidar_pose, Some(&vehicle), None)?;

        {
            let role_name = role_name.clone();
            let lidar_clone = lidar.clone();
            let mut frame_counter = 0..;

            lidar.listen(move |measure| {
                use flume::TrySendError as E;
                let measure: SemanticLidarMeasurement = measure.try_into().unwrap();
                let msg = LidarMessage {
                    frame_id: frame_counter.next().unwrap(),
                    role_name: role_name.clone(),
                    measure,
                    sub_outdir,
                };

                match lidar_tx.try_send(msg) {
                    Ok(_) => (),
                    Err(E::Disconnected(_)) => lidar_clone.stop(),
                    Err(E::Full(_)) => warn!("Lidar on {role_name} is unable to publish data"),
                }
            });
        }

        Ok(Self {
            _vehicle: vehicle,
            _lidar: lidar,
            role_name,
        })
    }
}
