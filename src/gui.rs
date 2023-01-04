use crate::{hack, message::GuiMessage};
use carla::{geom::Location, sensor::data::SemanticLidarDetection};
use kiss3d::{
    camera::{ArcBall, Camera},
    light::Light,
    nalgebra::{Point3, Vector3},
    planar_camera::PlanarCamera,
    post_processing::PostProcessingEffect,
    window::{State, Window},
};

use palette::{FromColor, Hsv, RgbHue, Srgb};
use std::{
    collections::HashMap,
    hash::Hash,
    sync::{
        atomic::{AtomicBool, Ordering::*},
        Arc,
    },
};

pub fn run_gui(gui_rx: flume::Receiver<GuiMessage>, is_terminated: Arc<AtomicBool>) {
    // Create a window object, which name is the binary name obtained from env!()
    let mut window = Window::new(env!("CARGO_BIN_NAME"));

    // Configure the camera perspective.
    // Change the up axis of the camera to (0, 0, 1).
    let mut camera = ArcBall::new(Point3::new(0.0, -80.0, 32.0), Point3::new(0.0, 0.0, 0.0));
    camera.set_up_axis(Vector3::new(0.0, 0.0, 1.0));

    window.set_light(Light::StickToCamera);
    let gui = Gui {
        gui_rx,
        cache: HashMap::new(),
        is_terminated,
        camera,
    };

    // Run GUI loop
    window.render_loop(gui);
}

struct Gui {
    /// Used to check termination signal
    is_terminated: Arc<AtomicBool>,

    /// The receiver used to receive rendering requests
    gui_rx: flume::Receiver<GuiMessage>,

    /// Stores cached points for each vehicle
    cache: HashMap<&'static str, VehicleCache>,

    /// The camera object required by cameras_and_effect().
    camera: ArcBall,
}

struct VehicleCache {
    pub color: Point3<f32>,
    pub points: Vec<Point3<f32>>,
}

impl State for Gui {
    /// The step() is called periodically.
    fn step(&mut self, window: &mut Window) {
        // Check termination signal
        if self.is_terminated.load(SeqCst) {
            window.close();
            return;
        }

        // Collect messages from the gui_rx.
        // Return if the channel is closed.
        let Some(msg_vec) = self.gather_msgs() else {
            window.close();
            return;
        };

        // Update GUI state.
        self.update(msg_vec);

        // Render the window according to the new state.
        self.render(window);
    }

    #[allow(clippy::type_complexity)]
    fn cameras_and_effect(
        &mut self,
    ) -> (
        Option<&mut dyn Camera>,
        Option<&mut dyn PlanarCamera>,
        Option<&mut dyn PostProcessingEffect>,
    ) {
        (Some(&mut self.camera), None, None)
    }
}

impl Gui {
    /// Gather queued messages from the input channel.
    ///
    /// Returns `None` if the channel is closed.
    fn gather_msgs(&self) -> Option<Vec<GuiMessage>> {
        let mut msg_vec = vec![];

        loop {
            //
            use flume::TryRecvError as E;
            match self.gui_rx.try_recv() {
                // Received a message
                Ok(msg) => msg_vec.push(msg),

                // The channel is closed. Close the GUI.
                Err(E::Disconnected) => {
                    return None;
                }

                // The channel remains but there is no more messages.
                Err(E::Empty) => break,
            };
        }

        Some(msg_vec)
    }

    /// Update GUI state according to a set of msgs.
    fn update(&mut self, msg_vec: Vec<GuiMessage>) {
        for msg in msg_vec {
            let GuiMessage {
                role_name,
                measure,
                transform,
            } = msg;
            let color = get_color_for_name(role_name);
            let transform = hack::convert_isometry3(&transform);

            // Convert point type.
            let points: Vec<_> = measure
                .as_slice()
                .iter()
                .map(|det| {
                    let SemanticLidarDetection {
                        point: Location { x, y, z },
                        // cos_inc_angle,
                        // object_idx,
                        // object_tag,
                        ..
                    } = *det;

                    let point = Point3::new(x, y, z);
                    transform * point
                })
                .collect();

            // Store data cache for this vehicle.
            let vehicle_cache = VehicleCache { color, points };
            self.cache.insert(role_name, vehicle_cache);
        }
    }

    fn render(&mut self, window: &mut Window) {
        self.cache.iter().for_each(|(_role_name, vehicle_cache)| {
            let VehicleCache { color, points } = vehicle_cache;

            points.iter().for_each(|position| {
                window.draw_point(position, color);
            });
        });
    }
}

/// Sample a deterministic color for a name
fn get_color_for_name(name: &str) -> Point3<f32> {
    let [r, g, b] = sample_color(name);
    Point3::new(r, g, b)
}

/// Samples a RGB array from the hash of input value.
fn sample_color<T>(seed: &T) -> [f32; 3]
where
    T: Hash + ?Sized,
{
    // Compute the hash for the seed
    let hash = fxhash::hash64(seed);

    // Generate HSV values
    let hue_degrees = (hash.wrapping_mul(79) % 360) as f32;
    let saturation = 1.0f32;
    let value = 1.0f32;

    // Convert to RGB
    let hsv = Hsv::new(RgbHue::from_degrees(hue_degrees), saturation, value);
    let (r, g, b) = Srgb::from_color(hsv).into_components();
    [r, g, b]
}
