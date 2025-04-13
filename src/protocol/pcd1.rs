use super::constants::*;
use super::header::parse_header;
use byteorder::{LittleEndian, ReadBytesExt};
use crc::{Crc, CRC_32_ISO_HDLC};
use std::io::Cursor;

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
