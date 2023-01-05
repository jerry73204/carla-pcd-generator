use crate::{
    config::Config,
    message::{GuiMessage, LidarMessage},
};
use anyhow::Result;
use carla::{
    client::{ActorBase, Sensor, Vehicle, World},
    sensor::data::SemanticLidarMeasurement,
};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use std::{
    ops::RangeFrom,
    path::{Path, PathBuf},
    sync::Arc,
};
use tracing::warn;

pub struct VehicleAgent {
    pub role_name: &'static str,
    _vehicle: Vehicle,
    _lidar: Sensor,
}

impl VehicleAgent {
    pub fn new(
        config: &Config,
        world: &mut World,
        role_name: &str,
        spawn_point: &Isometry3<f32>,
        aggregator_tx: flume::Sender<LidarMessage>,
        gui_tx: flume::Sender<GuiMessage>,
        sub_outdir: PathBuf,
    ) -> Result<Self> {
        let role_name: &'static str = Box::leak(Box::new(role_name.to_string()));
        let sub_outdir: &'static Path = Box::leak(Box::new(sub_outdir));

        // Create a vehicle
        let vehicle = world
            .actor_builder("vehicle.tesla.model3")?
            .set_attribute("role_name", role_name)?
            .spawn_vehicle(spawn_point)?;
        vehicle.set_autopilot(true);

        // Create a lidar sensor
        let lidar_pose = Isometry3 {
            rotation: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            translation: Translation3::new(0.0, 0.0, config.lidar.height_m),
        };
        let lidar = world
            .actor_builder("sensor.lidar.ray_cast_semantic")?
            .set_attribute("channels", &config.lidar.n_channels.to_string())?
            .set_attribute("points_per_second", "90000")?
            .set_attribute(
                "rotation_frequency",
                &config.lidar.rotation_freq.to_string(),
            )?
            .set_attribute("range", &config.lidar.range_m.to_string())?
            .spawn_sensor_opt(&lidar_pose, Some(&vehicle), None)?;

        // Register a callback on the lidar. Whenever the callback is
        // called, that is, a lidar scan is ready, it publishes a
        // message to the aggregator and the GUI.
        {
            let mut handler = LidarHandler {
                vehicle: vehicle.clone(),
                lidar: lidar.clone(),
                role_name,
                sub_outdir,
                aggregator_tx,
                gui_tx,
                frame_counter: 0..,
            };

            lidar.listen(move |measure| {
                let measure: SemanticLidarMeasurement = measure.try_into().unwrap();
                handler.process_msg(measure);
            });
        }

        Ok(Self {
            _vehicle: vehicle,
            _lidar: lidar,
            role_name,
        })
    }
}

struct LidarHandler {
    vehicle: Vehicle,
    lidar: Sensor,
    role_name: &'static str,
    sub_outdir: &'static Path,
    aggregator_tx: flume::Sender<LidarMessage>,
    gui_tx: flume::Sender<GuiMessage>,
    frame_counter: RangeFrom<usize>,
}

impl LidarHandler {
    pub fn process_msg(&mut self, measure: SemanticLidarMeasurement) {
        let measure = Arc::new(measure);
        let frame_id = self.frame_counter.next().unwrap();
        let transform = self.vehicle.transform();

        // Send a msg to the aggregator
        {
            let msg = LidarMessage {
                frame_id,
                role_name: self.role_name,
                measure: measure.clone(),
                sub_outdir: self.sub_outdir,
            };

            use flume::TrySendError as E;
            match self.aggregator_tx.try_send(msg) {
                Ok(_) => (),
                Err(E::Disconnected(_)) => self.lidar.stop(),
                Err(E::Full(_)) => warn!(
                    "Lidar on vehicle {} is unable to publish data",
                    self.role_name
                ),
            }
        }

        // Send a message to GUI
        {
            let msg = GuiMessage {
                role_name: self.role_name,
                measure,
                transform,
            };

            use flume::TrySendError as E;
            match self.gui_tx.try_send(msg) {
                Ok(_) => (),
                Err(E::Disconnected(_)) => self.lidar.stop(),
                Err(E::Full(_)) => warn!(
                    "Lidar on vehicle {} is unable to publish data",
                    self.role_name
                ),
            }
        }
    }
}
