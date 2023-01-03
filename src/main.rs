mod aggregator;
mod config;
mod data;
mod message;
mod vehicle;

use crate::vehicle::VehicleAgent;
use anyhow::{Context, Result};
use carla::{client::Client, rpc::EpisodeSettings};
use clap::Parser;
use itertools::Itertools;
use rand::prelude::*;
use std::{fs, num::NonZeroUsize, path::PathBuf, thread, time::Duration};
use tracing::{info, warn};

#[derive(Parser)]
struct Opts {
    #[clap(long, default_value = "127.0.0.1")]
    pub carla_host: String,
    #[clap(long, default_value = "2000")]
    pub carla_port: u16,
    #[clap(long)]
    pub map: Option<String>,
    #[clap(long, default_value = "1")]
    pub n_cars: NonZeroUsize,
    #[clap(short, long)]
    pub output_dir: PathBuf,
}

fn main() -> Result<()> {
    // Enable logging
    tracing_subscriber::fmt::init();

    // Parse command line arguments
    let opts = Opts::parse();

    // Prepare files and dirs
    fs::create_dir_all(&opts.output_dir).with_context(|| {
        format!(
            "output directory '{}' already exists",
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

    // Start data aggregator
    let (pcd_tx, pcd_rx) = flume::bounded(n_cars * 2);

    let _aggregator = thread::spawn(move || aggregator::run_aggregator(pcd_rx));

    // Spawn vehicles
    let _vehicles: Vec<VehicleAgent> = spawn_points
        .iter()
        .enumerate()
        .map(|(index, spawn_point)| {
            let role_name = format!("car_{:03}", index + 1);
            let sub_outdir = opts.output_dir.join(&role_name);
            fs::create_dir(&sub_outdir)?;

            let vehicle = VehicleAgent::new(
                &mut world,
                &role_name,
                spawn_point,
                pcd_tx.clone(),
                sub_outdir,
            )?;
            info!("Spawned vehicle {role_name}");
            anyhow::Ok(vehicle)
        })
        .try_collect()?;

    drop(pcd_tx);

    loop {
        world.tick();
    }
}
