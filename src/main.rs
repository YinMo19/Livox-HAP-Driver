use anyhow::Error;

use livox_lidar::LivoxLidar;
use rclrs::*;
mod livox_lidar;
mod protocol;
use std::env;

#[tokio::main]
async fn main() -> Result<(), Error> {
    let context_args = env::args().collect::<Vec<String>>();
    //     .find(|arg| arg.starts_with("__ns:="))
    //     .map(|arg| arg.replace("__ns:=", ""))
    //     .unwrap_or("/your_namespace".to_string());

    let mut executor = Context::new(context_args, InitOptions::default())?.create_basic_executor();
    // println!("{namespace}");
    // let mut executor = Context::default().create_basic_executor();
    let node = executor.create_node("hap_driver")?;

    // let node = executor.create_node("livox_lidar")?;
    log_info!(node.as_ref(), "Initializing Livox Lidar driver");

    let mut lidar = match LivoxLidar::new(node.clone()).await {
        Ok(lidar) => lidar,
        Err(e) => {
            log_error!(node.as_ref(), "Failed to initialize Livox Lidar: {}", e);
            return Err(e);
        }
    };

    tokio::spawn(async move {
        lidar.run().await;
    });

    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| err.into())
}
