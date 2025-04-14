use anyhow::{Error, Result};
use builtin_interfaces;
use rclrs::*;
use sensor_msgs::msg::PointCloud2;
use std::{net::Ipv4Addr, sync::Arc};
use tokio::net::UdpSocket;
use tokio::time::Duration;

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
            .default(Ipv4AddrWrapper("192.168.1.101".parse().unwrap()))
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
            "Initializing with parameters: dest_ip={}, port={}, batch_dot_num={}, line_num={}",
            dest_ip.get().0,
            local_port.get(),
            batch_dot_num.get(),
            line_num.get()
        );

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, local_port.get() as u16))
            .await
            .map_err(|e| {
                log_error!(node.as_ref(), "Failed to bind UDP socket: {}", e);
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
        pc_msg.point_step = 26; // x,y,z (float32) + intensity (float32) + tag (uint8) + resv (uint8) + timestamp (float64)
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
            need_start: true,
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

        log_info!(
            self.node.as_ref(),
            "Sending control frame to {}:{} with payload: {:02x?}",
            self.dest_ip,
            protocol::DEST_PORT,
            payload
        );

        match self
            .socket
            .send_to(&buf[..frame_size], (self.dest_ip, protocol::DEST_PORT))
            .await
        {
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

    fn publish_current_batch(&mut self) {
        let now = self.node.get_clock().now();
        self.pc_msg.header.stamp = builtin_interfaces::msg::Time {
            sec: (now.nsec / 1_000_000_000) as i32,
            nanosec: (now.nsec % 1_000_000_000) as u32,
        };

        // 更新消息的点数
        self.pc_msg.width = self.point_count as u32;
        self.pc_msg.row_step = self.pc_msg.width * self.pc_msg.point_step;

        match self.pc_pub.publish(self.pc_msg.clone()) {
            Ok(_) => log_debug!(
                self.node.as_ref(),
                "Published point cloud with {} points",
                self.point_count
            ),
            Err(e) => log_error!(self.node.as_ref(), "Failed to publish point cloud: {}", e),
        }

        // 重置点计数和清空缓冲区
        self.point_count = 0;

        // 重新设置消息宽度为最大批次大小
        self.pc_msg.width = self.batch_dot_num as u32;
        self.pc_msg.row_step = self.pc_msg.width * self.pc_msg.point_step;

        // 清空或重新分配缓冲区
        self.pc_msg.data = vec![0; (self.pc_msg.width * self.pc_msg.point_step) as usize];
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
        // 检查是否已经达到批次容量，确保在写入前先发布已有数据
        if self.point_count >= self.batch_dot_num {
            self.publish_current_batch();
        }

        // 重新计算偏移量（因为可能刚刚发布了数据）
        let offset = self.point_count * self.pc_msg.point_step as usize;
        let end_offset = offset + self.pc_msg.point_step as usize;

        // 确保不会越界
        if end_offset > self.pc_msg.data.len() {
            log_error!(
                self.node.as_ref(),
                "Buffer overflow prevented: {} > {}",
                end_offset,
                self.pc_msg.data.len()
            );
            // 紧急发布当前批次
            self.publish_current_batch();

            // 重新计算写入位置（不再递归调用）
            let new_offset = 0; // 因为刚刚清空了，新的偏移量从0开始

            // 写入数据
            self.pc_msg.data[new_offset..new_offset + 4].copy_from_slice(&x.to_le_bytes());
            self.pc_msg.data[new_offset + 4..new_offset + 8].copy_from_slice(&y.to_le_bytes());
            self.pc_msg.data[new_offset + 8..new_offset + 12].copy_from_slice(&z.to_le_bytes());
            self.pc_msg.data[new_offset + 12..new_offset + 16]
                .copy_from_slice(&intensity.to_le_bytes());
            self.pc_msg.data[new_offset + 16] = tag;
            self.pc_msg.data[new_offset + 17] = 0;
            self.pc_msg.data[new_offset + 18..new_offset + 26]
                .copy_from_slice(&timestamp.to_le_bytes());

            self.point_count = 1; // 设为1，因为我们刚刚添加了一个点
        } else {
            // 安全写入
            self.pc_msg.data[offset..offset + 4].copy_from_slice(&x.to_le_bytes());
            self.pc_msg.data[offset + 4..offset + 8].copy_from_slice(&y.to_le_bytes());
            self.pc_msg.data[offset + 8..offset + 12].copy_from_slice(&z.to_le_bytes());
            self.pc_msg.data[offset + 12..offset + 16].copy_from_slice(&intensity.to_le_bytes());
            self.pc_msg.data[offset + 16] = tag;
            self.pc_msg.data[offset + 17] = 0;
            self.pc_msg.data[offset + 18..offset + 26].copy_from_slice(&timestamp.to_le_bytes());

            self.point_count += 1;
        }
    }

    async fn process_pcd1(&mut self, header: &protocol::Header, points: &[protocol::Pcd1]) {
        for (i, point) in points.iter().enumerate() {
            let timestamp = header.timestamp as f64
                + (header.time_interval as f64 * i as f64) / 1_000_000_000.0;

            self.write_point_to_pc2(
                point.x as f32 / 1000.0,
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
            .unwrap()
            .get() as u64;

        loop {
            if self.need_start {
                log_info!(self.node.as_ref(), "Attempting to send control frame...");
                match self.send_control_frame().await {
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

                        if size < 28 {
                            log_warn!(
                                self.node.as_ref(),
                                "Packet too small ({} bytes), expected at least 28",
                                size
                            );
                            return Ok(0);
                        }

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
                                log_info!(self.node.as_ref(), "{:?}", points);
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
                Ok(Ok(size)) if size == usize::MAX => (),
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
