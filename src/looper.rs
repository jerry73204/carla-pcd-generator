use carla::client::World;
use std::{
    sync::atomic::{AtomicBool, Ordering::*},
    time::{Duration, Instant},
};
use tracing::{info, warn};

/// The looper runs a infinite loop that ticks the simulator.
///
/// It runs until termination signal is received.
pub fn run_looper(mut world: World, is_terminated: &AtomicBool) {
    let mut since = Instant::now();

    for frame_id in 0.. {
        // Check termination signal
        if is_terminated.load(SeqCst) {
            warn!("User interrupted");
            break;
        }

        // Print # of ticked frames every 10 secs
        if since.elapsed() >= Duration::from_secs(10) {
            info!("Ticked {} frames", frame_id + 1);
            since = Instant::now();
        }

        // Tick the simulator
        world.tick();
    }
}
