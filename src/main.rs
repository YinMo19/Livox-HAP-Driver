use anyhow::Error;
use async_std::task;

use livox_lidar::LivoxLidar;
use rclrs::*;
mod livox_lidar;
mod protocol;

#[async_std::main]
async fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("livox_lidar")?;
    log_info!(node.as_ref(), "Initializing Livox Lidar driver");

    let mut lidar = match LivoxLidar::new(node.clone()).await {
        Ok(lidar) => lidar,
        Err(e) => {
            log_error!(node.as_ref(), "Failed to initialize Livox Lidar: {}", e);
            return Err(e);
        }
    };

    task::spawn(async move {
        lidar.run().await;
    });

    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| err.into())
}
