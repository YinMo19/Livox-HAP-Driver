use anyhow::{Error, Result};
use async_std::net::UdpSocket;
use async_std::task;
use builtin_interfaces;
use rclrs::*;
use sensor_msgs::msg::PointCloud2;
use std::{net::Ipv4Addr, sync::Arc, time::Duration};

mod protocol {

    use byteorder::{LittleEndian, ReadBytesExt};
    use crc::{Crc, CRC_16_IBM_3740, CRC_32_ISO_HDLC};
    use std::io::Cursor;
    use std::io::Read;

    use std::sync::atomic::{AtomicU32, Ordering};

    pub const PUSH_KEY_VALUE: u16 = 0x0102;
    pub const PC_MSG_SIZE: usize = 1380;
    pub const DOT_NUM: usize = 96;
    pub const DEST_PORT: u16 = 56000;

    #[allow(unused)]
    #[derive(Debug)]
    pub struct Header {
        pub version: u8,
        pub length: u16,
        pub time_interval: u16,
        pub dot_num: u16,
        pub udp_cnt: u16,
        pub frame_cnt: u8,
        pub data_type: u8,
        pub time_type: u8,
        pub pack_info: u8,
        pub _padding: [u8; 11], // Added padding to match C++ struct
        pub crc32: u32,
        pub timestamp: u64,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct Pcd1 {
        pub x: i32,
        pub y: i32,
        pub z: i32,
        pub reflectivity: u8,
        pub tag: u8,
    }

    pub fn parse_header(data: &[u8]) -> Option<Header> {
        if data.len() < 28 {
            return None;
        }

        let mut rdr = Cursor::new(data);

        Some(Header {
            version: rdr.read_u8().ok()?,
            length: rdr.read_u16::<LittleEndian>().ok()?,
            time_interval: rdr.read_u16::<LittleEndian>().ok()?,
            dot_num: rdr.read_u16::<LittleEndian>().ok()?,
            udp_cnt: rdr.read_u16::<LittleEndian>().ok()?,
            frame_cnt: rdr.read_u8().ok()?,
            data_type: rdr.read_u8().ok()?,
            time_type: rdr.read_u8().ok()?,
            pack_info: rdr.read_u8().ok()?,
            _padding: {
                let mut pad = [0u8; 11];
                rdr.read_exact(&mut pad).ok()?;
                pad
            },
            crc32: rdr.read_u32::<LittleEndian>().ok()?,
            timestamp: rdr.read_u64::<LittleEndian>().ok()?,
        })
    }

    pub fn parse_pcd1(data: &[u8]) -> Option<Vec<Pcd1>> {
        if data.len() < PC_MSG_SIZE {
            return None;
        }

        let mut points = Vec::with_capacity(DOT_NUM);
        let mut rdr = Cursor::new(&data[28..]);

        for _ in 0..DOT_NUM {
            points.push(Pcd1 {
                x: rdr.read_i32::<LittleEndian>().ok()?,
                y: rdr.read_i32::<LittleEndian>().ok()?,
                z: rdr.read_i32::<LittleEndian>().ok()?,
                reflectivity: rdr.read_u8().ok()?,
                tag: rdr.read_u8().ok()?,
            });
            // Skip reserved bytes
            rdr.set_position(rdr.position() + 2);
        }

        Some(points)
    }

    pub fn check_header_pcd1(data: &[u8]) -> bool {
        if let Some(header) = parse_header(data) {
            if header.version != 0 {
                log::error!("Header version is not 0");
                return false;
            }
            if header.length as usize != PC_MSG_SIZE {
                log::error!("Header length is not {}", PC_MSG_SIZE);
                return false;
            }
            if header.dot_num as usize != DOT_NUM {
                log::error!("Dot number is not {}", DOT_NUM);
                return false;
            }
            if header.data_type != 1 {
                log::error!("Data type is not 1 (PCD1)");
                return false;
            }
            if (header.pack_info & 0x03) != 0 {
                log::error!("Pack info is not 0");
                return false;
            }

            let crc32 = Crc::<u32>::new(&CRC_32_ISO_HDLC);
            let mut digest = crc32.digest();
            let data_len = header.length as usize;
            if data.len() < data_len {
                return false;
            }
            digest.update(&data[28..data_len - 4]);
            let computed_crc = digest.finalize();

            if computed_crc != header.crc32 {
                log::error!(
                    "CRC32 mismatch: computed {:x}, expected {:x}",
                    computed_crc,
                    header.crc32
                );
                return false;
            }
            true
        } else {
            false
        }
    }

    pub fn build_control_frame(buf: &mut [u8], cmd_id: u16, data: &[u8]) -> usize {
        #[repr(C, packed)]
        struct FrameHeader {
            sof: u8,
            version: u8,
            length: u16,
            seq_num: u32,
            cmd_id: u16,
            cmd_type: u8,
            sender_type: u8,
            resv: [u8; 6],
            crc_16: u16,
            crc_32: u32,
        }

        static SEQ_NUM: AtomicU32 = AtomicU32::new(0);

        let header_size = std::mem::size_of::<FrameHeader>();
        let total_size = header_size + data.len();

        if buf.len() < total_size {
            return 0;
        }

        let seq_num = SEQ_NUM.fetch_add(1, Ordering::SeqCst);

        // Initialize header
        let mut header = FrameHeader {
            sof: 0xAA,
            version: 0,
            length: total_size as u16,
            seq_num,
            cmd_id,
            cmd_type: 0,
            sender_type: 0,
            resv: [0; 6],
            crc_16: 0,
            crc_32: 0,
        };

        // Calculate CRC16 (first 18 bytes of header)
        let crc16 = Crc::<u16>::new(&CRC_16_IBM_3740);
        let mut digest16 = crc16.digest();
        let header_bytes = unsafe {
            std::slice::from_raw_parts(
                &header as *const _ as *const u8,
                std::mem::size_of_val(&header) - 6, // minus crc_16 and crc_32
            )
        };
        digest16.update(header_bytes);
        header.crc_16 = digest16.finalize();

        // Calculate CRC32 (data part)
        let crc32 = Crc::<u32>::new(&CRC_32_ISO_HDLC);
        let mut digest32 = crc32.digest();
        digest32.update(data);
        header.crc_32 = digest32.finalize();

        // Copy header and data to buffer
        unsafe {
            std::ptr::copy_nonoverlapping(
                &header as *const _ as *const u8,
                buf.as_mut_ptr(),
                header_size,
            );
        }
        buf[header_size..header_size + data.len()].copy_from_slice(data);

        total_size
    }
}

#[allow(unused)]
struct LivoxLidar {
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
    need_start: bool, // Added to track connection state
}

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

impl LivoxLidar {
    async fn new(node: Arc<Node>) -> Result<Self> {
        // Initialize logger
        if let Err(e) = env_logger::try_init() {
            eprintln!("Failed to initialize logger: {}", e);
        }

        let dest_ip: rclrs::MandatoryParameter<Ipv4AddrWrapper> = node
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

        log::info!("Initializing Livox Lidar with frame_id: {}", frame_id);
        log::info!(
            "Parameters: batch_dot_num={}, line_num={}",
            batch_dot_num.get(),
            line_num.get()
        );

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, local_port.get() as u16))
            .await
            .map_err(|e| {
                log::error!("Failed to bind UDP socket: {}", e);
                Error::msg(e)
            })?;

        socket
            .connect((dest_ip.get().0, protocol::DEST_PORT))
            .await
            .map_err(|e| {
                log::error!("Failed to connect to Livox device: {}", e);
                Error::msg(e)
            })?;

        let pc_pub = node
            .create_publisher::<PointCloud2>("pc_raw".keep_last(10).transient_local())
            .map_err(|e| {
                log::error!("Failed to create publisher: {}", e);
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
                log::info!("Control frame sent successfully");
                self.need_start = false;
                Ok(())
            }
            Err(e) => {
                log::error!("Failed to send control frame: {}", e);
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
            log::warn!("Point cloud buffer full, discarding point");
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
                Ok(_) => log::debug!("Published point cloud with {} points", self.point_count),
                Err(e) => log::error!("Failed to publish point cloud: {}", e),
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

    async fn run(&mut self) {
        let mut buf = [0u8; protocol::PC_MSG_SIZE];

        loop {
            // Send control frame if needed
            if self.need_start {
                if let Err(e) = self.send_control_frame().await {
                    log::error!("Failed to send control frame: {}", e);
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
                            log::warn!("Received packet too small ({} bytes)", size);
                            return Ok(0);
                        }

                        if !protocol::check_header_pcd1(&buf) {
                            log::warn!("Invalid PCD1 header");
                            return Ok(0);
                        }

                        if let Some(header) = protocol::parse_header(&buf) {
                            if header.version != 0 || header.data_type != 1 {
                                log::warn!("Unsupported version or data type");
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
                Duration::from_millis(1000), // Use parameterized timeout
                recv_result,
            )
            .await
            {
                Ok(Ok(_)) => {} // Successfully received data
                Ok(Err(e)) => {
                    log::error!("Error receiving data: {}", e);
                    self.need_start = true;
                }
                Err(_) => {
                    log::warn!("Receive timeout");
                    self.need_start = true;
                }
            }
        }
    }
}

#[async_std::main]
async fn main() -> Result<(), Error> {
    // Initialize logger
    env_logger::init();

    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("livox_lidar")?;

    let mut lidar = match LivoxLidar::new(node).await {
        Ok(lidar) => lidar,
        Err(e) => {
            log::error!("Failed to initialize Livox Lidar: {}", e);
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
