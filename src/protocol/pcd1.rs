use super::constants::*;
use super::header::Header;
use byteorder::{LittleEndian, ReadBytesExt};
use crc::{Crc, CRC_32_ISO_HDLC};
use rclrs::*;
use std::io::Cursor;
use std::sync::Arc;

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct LivoxPointXyzrtlt {
    pub x: f32, // 单位: m
    pub y: f32,
    pub z: f32,
    pub reflectivity: f32,
    pub tag: u8,
    pub resv: u8,
    pub timestamp: f64, // 8字节时间戳
}

#[derive(Debug, Clone, Copy)]
pub struct Pcd1 {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub reflectivity: u8,
    pub tag: u8,
}

pub fn parse_pcd1(data: &[u8]) -> Option<Vec<Pcd1>> {
    if data.len() < PC_MSG_SIZE {
        return None;
    }

    // Each Pcd1 point is 14 bytes (3x i32 + 2x u8 + 2 padding bytes)
    const POINT_SIZE: usize = 14;
    let points_data = &data[36..]; // Skip header
    let num_points = points_data.len() / POINT_SIZE;

    let mut points = Vec::with_capacity(num_points);
    let mut offset = 0;

    for _ in 0..num_points {
        if offset + POINT_SIZE > points_data.len() {
            break;
        }

        let point_slice = &points_data[offset..offset + POINT_SIZE];
        let mut rdr = Cursor::new(point_slice);

        points.push(Pcd1 {
            x: rdr.read_i32::<LittleEndian>().ok()?,
            y: rdr.read_i32::<LittleEndian>().ok()?,
            z: rdr.read_i32::<LittleEndian>().ok()?,
            reflectivity: rdr.read_u8().ok()?,
            tag: rdr.read_u8().ok()?,
        });

        offset += POINT_SIZE;
    }

    Some(points)
}

pub fn check_header_pcd1(data: &[u8], header: &Header, node: Arc<Node>) -> bool {
    if header.version != 0 {
        log_error!(node.as_ref(), "Invalid header version: {}", header.version);
        return false;
    }
    if header.length as usize != PC_MSG_SIZE {
        log_error!(node.as_ref(), "Header length is not {}", PC_MSG_SIZE);
        return false;
    }
    if header.dot_num as usize != DOT_NUM {
        log_error!(node.as_ref(), "Dot number is not {}", DOT_NUM);
        return false;
    }
    if header.data_type != 1 {
        log_error!(node.as_ref(), "Data type is not 1 (PCD1)");
        return false;
    }
    if (header.pack_info & 0x03) != 0 {
        log_error!(node.as_ref(), "Pack info is not 0");
        return false;
    }

    let crc32 = Crc::<u32>::new(&CRC_32_ISO_HDLC);
    let mut digest = crc32.digest();
    let data_len = header.length as usize;
    if data.len() < data_len {
        log_error!(node.as_ref(), "data length is shorter than {}", data_len);
        return false;
    }
    digest.update(&data[28..data_len]);
    let computed_crc = digest.finalize();

    if computed_crc != header.crc32 {
        log_error!(
            node.as_ref(),
            "CRC32 mismatch: computed {:x}, expected {:x}",
            computed_crc,
            header.crc32
        );
        return false;
    }
    true
}
