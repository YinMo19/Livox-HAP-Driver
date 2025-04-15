use crate::protocol;
use anyhow::{Error, Result};
use builtin_interfaces;
use rclrs::*;
use sensor_msgs::msg::PointCloud2;
use std::{net::Ipv4Addr, sync::Arc};
use tokio::net::UdpSocket;
use tokio::time::Duration;

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
    local_ip: Ipv4Addr,
    dest_udp_port: u16,
    local_port: u16,
    batch_dot_num: usize,
    line_num: usize,
    frame_id: String,
    pc_pub: Arc<Publisher<PointCloud2>>,
    pc_msg: PointCloud2,
    point_index: usize,
    need_start: bool,
    time_out: u64,
}

impl LivoxLidar {
    pub async fn new(node: Arc<Node>) -> Result<Self> {
        let dest_ip = node
            .declare_parameter("dest_ip")
            .default(Ipv4AddrWrapper("192.168.1.101".parse().unwrap()))
            .mandatory()
            .map_err(|e| Error::msg(e))?;

        let udp_range = ParameterRange {
            lower: Some(1025),
            upper: Some(65535),
            step: Some(1),
        };

        let dest_udp_port = node
            .declare_parameter("dest_udp_port")
            .range(udp_range)
            .default(57000)
            .mandatory()
            .map_err(|e| Error::msg(e))?;

        let local_ip = node
            .declare_parameter("local_ip")
            .default(Ipv4AddrWrapper("192.168.1.50".parse().unwrap()))
            .mandatory()
            .map_err(|e| Error::msg(e))?;

        let local_port = node
            .declare_parameter("local_udp_port")
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

        let timeout_ms = node
            .declare_parameter("timeout_ms")
            .default(1000)
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
            "Initializing with parameters: dest_ip={}, dest_port={}, batch_dot_num={}, line_num={}, local_ip={}, local_port={}",
            dest_ip.get().0,
            dest_udp_port.get(),
            batch_dot_num.get(),
            line_num.get(),
            local_ip.get().0,
            local_port.get()
        );

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, local_port.get() as u16))
            .await
            .map_err(|e| {
                log_error!(node.as_ref(), "Failed to bind UDP socket: {}", e);
                Error::msg(e)
            })?;

        let pc_pub = node
            .create_publisher::<PointCloud2>(
                "pc_raw"
                    .keep_last(10)
                    .best_effort()
                    .durability(QoSDurabilityPolicy::Volatile), // Larger queue size
            )
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
        pc_msg.point_step = 26; // x,y,z (float32) + intensity (float32) + tag (uint8) + resv (uint8) + timestamp (float64)
        pc_msg.row_step = pc_msg.width * pc_msg.point_step;
        pc_msg.is_dense = true;
        pc_msg.data = vec![0; pc_msg.row_step as usize];

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

        Ok(Self {
            node,
            socket,
            dest_ip: dest_ip.get().0,
            local_ip: local_ip.get().0,
            dest_udp_port: dest_udp_port.get() as u16,
            local_port: local_port.get() as u16,
            batch_dot_num: batch_dot_num.get() as usize,
            line_num: line_num.get() as usize,
            frame_id,
            pc_pub,
            pc_msg,
            point_index: 0, // 初始化索引为0
            need_start: true,
            time_out: timeout_ms.get() as u64,
        })
    }

    fn write_point_to_pc2(&mut self, point: &protocol::LivoxPointXyzrtlt) {
        let offset = self.point_index * std::mem::size_of::<protocol::LivoxPointXyzrtlt>();
        let bytes = unsafe {
            std::slice::from_raw_parts(
                point as *const _ as *const u8,
                std::mem::size_of::<protocol::LivoxPointXyzrtlt>(),
            )
        };
        self.pc_msg.data[offset..offset + 26].copy_from_slice(bytes);
        self.point_index += 1;

        // 严格按批次发布（不检查越界，假设缓冲区足够）
        if self.point_index >= self.batch_dot_num {
            self.publish_current_batch();
        }
    }

    fn publish_current_batch(&mut self) {
        if self.point_index == 0 {
            return; // 没有数据时不发布
        }

        let now = self.node.get_clock().now();
        self.pc_msg.header.stamp = builtin_interfaces::msg::Time {
            sec: (now.nsec / 1_000_000_000) as i32,
            nanosec: (now.nsec % 1_000_000_000) as u32,
        };

        // 更新实际点数
        self.pc_msg.width = self.point_index as u32;
        self.pc_msg.row_step = self.pc_msg.width * self.pc_msg.point_step;

        // 发布点云
        match self.pc_pub.publish(self.pc_msg.clone()) {
            Ok(_) => log_debug!(
                self.node.as_ref(),
                "Published point cloud with {} points",
                self.point_index
            ),
            Err(e) => log_error!(self.node.as_ref(), "Failed to publish point cloud: {}", e),
        }

        // 重置状态
        self.pc_msg.data.fill(0);
        self.point_index = 0;
        self.pc_msg.width = self.batch_dot_num as u32;
        self.pc_msg.row_step = self.pc_msg.width * self.pc_msg.point_step;
    }

    pub async fn set_pointcloud_host(
        &mut self,
        config: protocol::PointCloudHostConfig,
    ) -> Result<()> {
        // 验证IP地址有效性
        if config.dest_ip.is_unspecified() || config.dest_ip.is_broadcast() {
            return Err(Error::msg("invalid ipv4 addr"));
        }

        let mut buf = [0u8; 64];
        let config_bytes = config.to_bytes();

        let message = protocol::KeyValueMessage {
            key_num: 1,
            rsvd: 0,
            entries: vec![(
                protocol::KeyValueEntry {
                    key: 0x0006, // pointcloud_host_ipcfg 的键
                    length: 8,   // 固定8字节
                },
                config_bytes.to_vec(),
            )],
        };

        let payload = message.to_bytes();
        let frame_size =
            protocol::build_control_frame(&mut buf, protocol::SETTING_KEY_VALUE, &payload);

        if frame_size == 0 {
            return Err(Error::msg("Failed to build control frame"));
        }

        log_info!(
            self.node.as_ref(),
            "Setting point cloud host config: Dest={}:{}, SrcPort={}",
            config.dest_ip,
            config.dest_port,
            config.src_port
        );

        log_info!(
            self.node.as_ref(),
            "Sending control frame to {}:{} with payload: {:02x?}",
            self.dest_ip,
            protocol::DEST_PORT,
            payload
        );

        self.send_control_frame(&buf[..frame_size]).await
    }

    async fn send_control_ws_start_frame(&mut self) -> Result<()> {
        let mut buf = [0u8; 64];

        let message = protocol::KeyValueMessage {
            key_num: 1,
            rsvd: 0,
            entries: vec![(
                protocol::KeyValueEntry {
                    key: 0x001a,
                    length: 1,
                },
                vec![0x01_u8],
            )],
        };

        let payload = message.to_bytes();

        let frame_size =
            protocol::build_control_frame(&mut buf, protocol::PUSH_KEY_VALUE, &payload);

        if frame_size == 0 {
            return Err(Error::msg("Failed to build control frame"));
        }

        log_info!(
            self.node.as_ref(),
            "Sending control frame to {}:{} with payload: {:02x?}",
            self.dest_ip,
            protocol::DEST_PORT,
            payload
        );

        self.send_control_frame(&buf[..frame_size]).await
    }

    async fn send_control_frame(&mut self, frame: &[u8]) -> Result<()> {
        match self
            .socket
            .send_to(frame, (self.dest_ip, protocol::DEST_PORT))
            .await
        {
            Ok(_) => {
                log_info!(self.node.as_ref(), "Control frame sent successfully");
                Ok(())
            }
            Err(e) => {
                log_error!(self.node.as_ref(), "Failed to send frame: {}", e);
                Err(Error::msg("Failed to build control frame"))
            }
        }
    }

    async fn process_pcd1(&mut self, header: &protocol::Header, points: &[protocol::Pcd1]) {
        for (i, point) in points.iter().enumerate() {
            let timestamp = (header.timestamp + header.time_interval as u64 * i as u64) as f64;

            // println!(
            //     "Raw point [{i}]: x={}, y={}, z={}",
            //     point.x, point.y, point.z
            // );
            self.write_point_to_pc2(&protocol::LivoxPointXyzrtlt {
                x: point.x as f32 / 1000.0,
                y: point.y as f32 / 1000.0,
                z: point.z as f32 / 1000.0,
                reflectivity: point.reflectivity as f32,
                tag: point.tag,
                resv: 0,
                timestamp,
            });
        }
    }

    pub async fn run(&mut self) {
        let mut buf = [0u8; protocol::PC_MSG_SIZE];
        let timeout_ms = self.time_out;

        loop {
            if self.need_start {
                log_info!(self.node.as_ref(), "Attempting to send control frame...");
                let config = protocol::PointCloudHostConfig {
                    dest_ip: self.local_ip,
                    dest_port: self.local_port,
                    ..Default::default()
                };

                match self.set_pointcloud_host(config).await {
                    Ok(_) => {}
                    Err(e) => {
                        log_error!(self.node.as_ref(), "Failed to send control frame: {}", e);
                        tokio::time::sleep(Duration::from_millis(100)).await;
                        continue;
                    }
                }

                match self.send_control_ws_start_frame().await {
                    Ok(_) => {
                        self.need_start = false;
                    }
                    Err(e) => {
                        log_error!(self.node.as_ref(), "Failed to send control frame: {}", e);
                        tokio::time::sleep(Duration::from_millis(100)).await;
                        continue;
                    }
                }
            }

            let recv_fut = async {
                match self.socket.recv_from(&mut buf).await {
                    Ok((size, _src_addr)) => {
                        if size > 0 {
                            log_debug!(
                                self.node.as_ref(),
                                "Packet header: {:02x?}",
                                &buf[..size.min(32)]
                            );
                        }

                        // if size < 28 {
                        //     log_warn!(
                        //         self.node.as_ref(),
                        //         "Packet too small ({} bytes), expected at least 28",
                        //         size
                        //     );
                        //     return Ok(0);
                        // }

                        let header = match protocol::parse_header(&buf) {
                            Some(h) => h,
                            None => {
                                log_error!(self.node.as_ref(), "Failed to parse header");
                                return Ok(0);
                            }
                        };

                        if header.version == 0xAA {
                            return Ok(usize::MAX);
                        }

                        if !protocol::check_header_pcd1(&buf, &header, self.node.clone()) {
                            log_warn!(self.node.as_ref(), "Invalid PCD1 header");
                            return Ok(0);
                        }

                        if header.version != 0 || header.data_type != 1 {
                            log_warn!(
                                self.node.as_ref(),
                                "Unsupported protocol: version={}, data_type={}",
                                header.version,
                                header.data_type
                            );
                            return Ok(0);
                        }

                        match protocol::parse_pcd1(&buf) {
                            Some(points) => {
                                self.process_pcd1(&header, &points).await;
                                // log_info!(self.node.as_ref(), "{:?}", points);
                                Ok(size)
                            }
                            None => {
                                log_error!(self.node.as_ref(), "Failed to parse points data");
                                Ok(0)
                            }
                        }
                    }
                    Err(e) => Err(e),
                }
            };

            match tokio::time::timeout(Duration::from_millis(timeout_ms), recv_fut).await {
                Ok(Ok(size)) if size > 0 => (),
                Ok(Ok(_)) => self.need_start = true,
                Ok(Err(e)) => {
                    log_error!(self.node.as_ref(), "Receive error: {}", e);
                    self.need_start = true;
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
                Err(_) => {
                    log_warn!(self.node.as_ref(), "Receive timeout after {}ms", timeout_ms);
                    self.need_start = true;
                }
            }
        }
    }
}
