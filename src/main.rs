mod aggregator;
mod config;
mod consts;
mod data;
mod gui;
mod hack;
mod looper;
mod message;
mod vehicle;

use crate::{config::Config, consts::DEFAULT_CONFIG, vehicle::VehicleAgent};
use anyhow::{Context, Result};
use carla::{client::Client, rpc::EpisodeSettings};
use clap::Parser;
use itertools::Itertools;
use rand::prelude::*;
use std::{
    fs,
    num::NonZeroUsize,
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering::*},
        Arc,
    },
    thread,
    time::Duration,
};
use tracing::{info, warn};

#[derive(Parser)]
struct Opts {
    /// Carla server address.
    #[clap(long, default_value = "127.0.0.1")]
    pub carla_host: String,

    /// Carla server port.
    #[clap(long, default_value = "2000")]
    pub carla_port: u16,

    /// Load the map if specified.
    #[clap(long)]
    pub map: Option<String>,

    /// Number of vehicles.
    #[clap(long, default_value = "1")]
    pub n_cars: NonZeroUsize,

    /// Configuration file.
    #[clap(long)]
    pub config: Option<PathBuf>,

    /// Output directory to store point cloud files.
    ///
    /// The directory must not exist.
    #[clap(short, long)]
    pub output_dir: PathBuf,
}

fn main() -> Result<()> {
    // Enable logging
    tracing_subscriber::fmt::init();

    // Parse command line arguments
    let opts = Opts::parse();
    let config: &'static Config = match &opts.config {
        Some(path) => {
            let text = fs::read_to_string(path)?;
            let config: Config = json5::from_str(&text)?;
            Box::leak(Box::new(config))
        }
        None => &DEFAULT_CONFIG,
    };

    // Prepare files and dirs
    fs::create_dir(&opts.output_dir).with_context(|| {
        format!(
            "Unable to create output directory '{}'",
            opts.output_dir.display()
        )
    })?;

    // Create random generator
    let mut rng = rand::thread_rng();

    // Connect to Carla simulator
    let client = Client::connect(&opts.carla_host, opts.carla_port, None);

    // Load new map if requested
    if let Some(map_name) = &opts.map {
        client.load_world(map_name);
    } else {
        client.reload_world();
    }

    // Enable synchronous mode and fixed time step
    let mut world = client.world();

    let settings = EpisodeSettings {
        fixed_delta_seconds: Some(0.05),
        synchronous_mode: true,
        ..world.settings()
    };
    world.apply_settings(&settings, Duration::from_secs(1));

    // Get spawn points
    let map = world.map();
    let mut spawn_points: Vec<_> = map.recommended_spawn_points().iter().collect();

    // Determine number of cars
    let n_cars = {
        let wanted_n_cars = opts.n_cars.get();
        let n_spawn_points = spawn_points.len();

        if wanted_n_cars > n_spawn_points {
            warn!(
                "n_cars (wanted_n_cars) exceeds max number of spawn points ({n_spawn_points}). \
                 Only {n_spawn_points} vehicles will be spawned."
            );
            n_spawn_points
        } else {
            wanted_n_cars
        }
    };

    // Shuffle spawn points
    let (spawn_points, _) = spawn_points.partial_shuffle(&mut rng, n_cars);

    // Create channels
    let (lidar_tx, lidar_rx) = flume::bounded(n_cars);
    let (gui_tx, gui_rx) = flume::bounded(n_cars);

    // Start data aggregator
    let aggregator = thread::spawn(move || aggregator::run_aggregator(lidar_rx));

    // Spawn vehicles
    let vehicles: Vec<VehicleAgent> = spawn_points
        .iter()
        .enumerate()
        .map(|(index, spawn_point)| -> Result<_> {
            let role_name = format!("car_{:03}", index + 1);
            let sub_outdir = opts.output_dir.join(&role_name);
            fs::create_dir(&sub_outdir)?;

            let vehicle = VehicleAgent::new(
                config,
                &mut world,
                &role_name,
                spawn_point,
                lidar_tx.clone(),
                gui_tx.clone(),
                sub_outdir,
            )?;
            info!("Spawned vehicle {role_name}");
            Ok(vehicle)
        })
        .try_collect()?;

    // Drop the sender that is no longer used.
    drop(lidar_tx);
    drop(gui_tx);

    // Set Ctrl-C handler
    let is_terminated = Arc::new(AtomicBool::new(false));
    {
        let is_terminated = is_terminated.clone();
        ctrlc::set_handler(move || {
            is_terminated.store(true, SeqCst);
        })?;
    }

    // Run the looper that ticks the simulator
    info!("Simulation started");
    let looper = {
        let is_terminated = is_terminated.clone();
        thread::spawn(move || looper::run_looper(world, &is_terminated))
    };

    // Run GUI. Windows and Mac OSX restricts that GUI must run in
    // main thread.  Hence, the GUI is launched in main() without
    // putting it into a thread.
    gui::run_gui(gui_rx, is_terminated);

    // Finalize
    drop(vehicles);
    aggregator.join().unwrap()?;
    looper.join().unwrap();

    Ok(())
}
