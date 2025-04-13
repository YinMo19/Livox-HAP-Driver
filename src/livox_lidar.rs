use anyhow::{Error, Result};
use async_std::net::UdpSocket;
use async_std::task;
use builtin_interfaces;
use rclrs::*;
use sensor_msgs::msg::PointCloud2;
use std::{net::Ipv4Addr, sync::Arc, time::Duration};

use crate::protocol;

#[derive(Debug, Clone)]
struct Ipv4AddrWrapper(Ipv4Addr);

impl TryFrom<ParameterValue> for Ipv4AddrWrapper {
    type Error = String;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::String(s) => s
                .parse::<Ipv4Addr>()
                .map(Ipv4AddrWrapper)
                .map_err(|e| format!("Invalid IPv4 address: {}", e)),
            _ => Err("Expected string parameter for IPv4 address".to_string()),
        }
    }
}

impl From<Ipv4AddrWrapper> for ParameterValue {
    fn from(wrapper: Ipv4AddrWrapper) -> Self {
        ParameterValue::String(Arc::from(wrapper.0.to_string()))
    }
}

impl ParameterVariant for Ipv4AddrWrapper {
    type Range = ();

    fn kind() -> ParameterKind {
        ParameterKind::String
    }
}

#[allow(unused)]
pub struct LivoxLidar {
    node: Arc<Node>,
    socket: UdpSocket,
    dest_ip: Ipv4Addr,
    local_port: u16,
    batch_dot_num: usize,
    line_num: usize,
    frame_id: String,
    pc_pub: Arc<Publisher<PointCloud2>>,
    pc_msg: PointCloud2,
    point_count: usize,
    need_start: bool,
}

impl LivoxLidar {
    pub async fn new(node: Arc<Node>) -> Result<Self> {
        let dest_ip = node
            .declare_parameter("dest_ip")
            .default(Ipv4AddrWrapper("192.168.1.1".parse().unwrap()))
            .mandatory()
            .map_err(|e| Error::msg(e))?;

        let local_port = node
            .declare_parameter("udp_port")
            .default(57000)
            .mandatory()
            .map_err(|e| Error::msg(e))?;

        let batch_dot_num = node
            .declare_parameter("batch_dot_num")
            .default(9600)
            .mandatory()
            .map_err(|e| Error::msg(e))?;

        let line_num = node
            .declare_parameter("lidar_line")
            .default(6)
            .mandatory()
            .map_err(|e| Error::msg(e))?;

        // Validate parameters
        if line_num.get() < 1 || line_num.get() > 128 {
            return Err(Error::msg("Invalid line_num, must be between 1 and 128"));
        }

        let frame_id = match node.namespace().rfind('/') {
            Some(pos) => format!("{}_frame", &node.namespace()[pos + 1..]),
            None => "default_lidar_frame".to_string(),
        };

        log_info!(node.as_ref(), "Initializing with frame_id: {}", frame_id);

        log_info!(
            node.as_ref(),
            "Parameters: batch_dot_num={}, line_num={}",
            batch_dot_num.get(),
            line_num.get()
        );

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, local_port.get() as u16))
            .await
            .map_err(|e| {
                log_error!(node.as_ref(), "Failed to bind UDP socket: {}", e);
                Error::msg(e)
            })?;

        socket
            .connect((dest_ip.get().0, protocol::DEST_PORT))
            .await
            .map_err(|e| {
                log_error!(node.as_ref(), "Failed to connect to Livox device: {}", e);
                Error::msg(e)
            })?;

        let pc_pub = node
            .create_publisher::<PointCloud2>("pc_raw".keep_last(10).transient_local())
            .map_err(|e| {
                log_error!(node.as_ref(), "Failed to create publisher: {}", e);
                Error::msg(e)
            })?;

        // Initialize point cloud message
        let mut pc_msg = PointCloud2::default();
        pc_msg.header.frame_id = frame_id.clone();
        pc_msg.height = 1;
        pc_msg.width = batch_dot_num.get() as u32;
        pc_msg.is_bigendian = false;
        pc_msg.point_step = 24; // x,y,z (float32) + intensity (float32) + tag (uint8) + resv (uint8) + timestamp (float64)
        pc_msg.row_step = pc_msg.width * pc_msg.point_step;
        pc_msg.is_dense = true;

        // Set fields
        let mut fields = Vec::new();
        fields.push(sensor_msgs::msg::PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: sensor_msgs::msg::PointField::FLOAT32,
            count: 1,
        });
        fields.push(sensor_msgs::msg::PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: sensor_msgs::msg::PointField::FLOAT32,
            count: 1,
        });
        fields.push(sensor_msgs::msg::PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: sensor_msgs::msg::PointField::FLOAT32,
            count: 1,
        });
        fields.push(sensor_msgs::msg::PointField {
            name: "intensity".to_string(),
            offset: 12,
            datatype: sensor_msgs::msg::PointField::FLOAT32,
            count: 1,
        });
        fields.push(sensor_msgs::msg::PointField {
            name: "tag".to_string(),
            offset: 16,
            datatype: sensor_msgs::msg::PointField::UINT8,
            count: 1,
        });
        fields.push(sensor_msgs::msg::PointField {
            name: "resv".to_string(),
            offset: 17,
            datatype: sensor_msgs::msg::PointField::UINT8,
            count: 1,
        });
        fields.push(sensor_msgs::msg::PointField {
            name: "timestamp".to_string(),
            offset: 18,
            datatype: sensor_msgs::msg::PointField::FLOAT64,
            count: 1,
        });
        pc_msg.fields = fields;
        pc_msg.data = vec![0; (pc_msg.width * pc_msg.point_step) as usize];

        Ok(Self {
            node,
            socket,
            dest_ip: dest_ip.get().0,
            local_port: local_port.get() as u16,
            batch_dot_num: batch_dot_num.get() as usize,
            line_num: line_num.get() as usize,
            frame_id,
            pc_pub,
            pc_msg,
            point_count: 0,
            need_start: true, // Start with need_start true to send initial control frame
        })
    }

    async fn send_control_frame(&mut self) -> Result<()> {
        let mut buf = [0u8; 64];
        let payload = [
            0x01, 0x02, 0x00, 0x1a, 0x01, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ];

        let frame_size =
            protocol::build_control_frame(&mut buf, protocol::PUSH_KEY_VALUE, &payload);

        if frame_size == 0 {
            return Err(Error::msg("Failed to build control frame"));
        }

        match self.socket.send(&buf[..frame_size]).await {
            Ok(_) => {
                log_info!(self.node.as_ref(), "Control frame sent successfully");
                self.need_start = false;
                Ok(())
            }
            Err(e) => {
                log_error!(self.node.as_ref(), "Failed to send control frame: {}", e);
                self.need_start = true;
                Err(Error::msg(e))
            }
        }
    }

    fn write_point_to_pc2(
        &mut self,
        x: f32,
        y: f32,
        z: f32,
        intensity: f32,
        tag: u8,
        timestamp: f64,
    ) {
        let offset = self.point_count * self.pc_msg.point_step as usize;

        // Ensure we don't write out of bounds
        if offset + self.pc_msg.point_step as usize > self.pc_msg.data.len() {
            log_warn!(
                self.node.as_ref(),
                "Point cloud buffer full, discarding point"
            );
            return;
        }

        // Write x,y,z (float32)
        self.pc_msg.data[offset..offset + 4].copy_from_slice(&x.to_le_bytes());
        self.pc_msg.data[offset + 4..offset + 8].copy_from_slice(&y.to_le_bytes());
        self.pc_msg.data[offset + 8..offset + 12].copy_from_slice(&z.to_le_bytes());

        // Write intensity (float32)
        self.pc_msg.data[offset + 12..offset + 16].copy_from_slice(&intensity.to_le_bytes());

        // Write tag (uint8)
        self.pc_msg.data[offset + 16] = tag;

        // Write resv (uint8)
        self.pc_msg.data[offset + 17] = 0;

        // Write timestamp (float64)
        self.pc_msg.data[offset + 18..offset + 26].copy_from_slice(&timestamp.to_le_bytes());

        self.point_count += 1;

        // Publish if point cloud is full
        if self.point_count >= self.batch_dot_num {
            let now = self.node.get_clock().now();
            self.pc_msg.header.stamp = builtin_interfaces::msg::Time {
                sec: (now.nsec / 1_000_000_000) as i32,
                nanosec: (now.nsec % 1_000_000_000) as u32,
            };

            match self.pc_pub.publish(self.pc_msg.clone()) {
                Ok(_) => log_debug!(
                    self.node.as_ref(),
                    "Published point cloud with {} points",
                    self.point_count
                ),
                Err(e) => log_error!(self.node.as_ref(), "Failed to publish point cloud: {}", e),
            }

            // Reset for next batch
            self.point_count = 0;
            // Clear the data buffer
            self.pc_msg.data.fill(0);
        }
    }

    async fn process_pcd1(&mut self, header: &protocol::Header, points: &[protocol::Pcd1]) {
        for (i, point) in points.iter().enumerate() {
            // Convert timestamp (assuming it's in nanoseconds)
            let timestamp = header.timestamp as f64
                + (header.time_interval as f64 * i as f64) / 1_000_000_000.0;

            self.write_point_to_pc2(
                point.x as f32 / 1000.0, // Convert mm to m
                point.y as f32 / 1000.0,
                point.z as f32 / 1000.0,
                point.reflectivity as f32,
                point.tag,
                timestamp,
            );
        }
    }

    pub async fn run(&mut self) {
        let mut buf = [0u8; protocol::PC_MSG_SIZE];

        let timeout_ms = self
            .node
            .declare_parameter("timeout_ms")
            .default(1000)
            .mandatory()
            .unwrap();

        loop {
            // Send control frame if needed
            if self.need_start {
                if let Err(e) = self.send_control_frame().await {
                    log_error!(self.node.as_ref(), "Failed to send control frame: {}", e);
                    // Wait before retrying
                    task::sleep(Duration::from_millis(100)).await;
                    continue;
                }
            }

            // Set receive timeout
            let recv_result = async {
                match self.socket.recv_from(&mut buf).await {
                    Ok((size, _)) => {
                        if size < 28 {
                            log_warn!(
                                self.node.as_ref(),
                                "Received packet too small ({} bytes)",
                                size
                            );
                            return Ok(0);
                        }

                        if !protocol::check_header_pcd1(&buf, self.node.clone()) {
                            log_warn!(self.node.as_ref(), "Invalid PCD1 header");
                            return Ok(0);
                        }

                        if let Some(header) = protocol::parse_header(&buf) {
                            if header.version != 0 || header.data_type != 1 {
                                log_warn!(self.node.as_ref(), "Unsupported version or data type");
                                return Ok(0);
                            }

                            if let Some(points) = protocol::parse_pcd1(&buf) {
                                self.process_pcd1(&header, &points).await;
                            }
                        }
                        Ok(size)
                    }
                    Err(e) => Err(e),
                }
            };

            // Add timeout
            match async_std::future::timeout(
                Duration::from_millis(timeout_ms.get() as u64), // Use parameterized timeout
                recv_result,
            )
            .await
            {
                Ok(Ok(_)) => {} // Successfully received data
                Ok(Err(e)) => {
                    log_error!(self.node.as_ref(), "Error receiving data: {}", e);
                    self.need_start = true;
                }
                Err(_) => {
                    log_warn!(self.node.as_ref(), "Receive timeout");
                    self.need_start = true;
                }
            }
        }
    }
}
