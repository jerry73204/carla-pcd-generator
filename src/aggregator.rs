use crate::{data::PcdPoint, message::LidarMessage};
use anyhow::Result;
use pcd_rs::{DataKind, WriterInit};
use rayon::prelude::*;

pub fn run_aggregator(lidar_rx: flume::Receiver<LidarMessage>) -> Result<()> {
    lidar_rx
        .into_iter()
        .par_bridge()
        .try_for_each(process_msg)?;
    Ok(())
}

fn process_msg(msg: LidarMessage) -> Result<()> {
    let LidarMessage {
        measure,
        frame_id,
        sub_outdir,
        ..
    } = msg;
    let points: Vec<_> = measure.as_slice().par_iter().map(PcdPoint::from).collect();
    let output_path = sub_outdir.join(format!("{frame_id:05}.pcd"));

    // let height = measure.channel_count() as u64;
    // let width = measure.len() as u64 / height;

    let mut writer = WriterInit {
        width: measure.len() as u64,
        height: 1,
        viewpoint: Default::default(),
        data_kind: DataKind::Binary,
        schema: None,
    }
    .create(output_path)?;

    for point in points {
        writer.push(&point)?;
    }

    writer.finish()?;

    Ok(())
}
