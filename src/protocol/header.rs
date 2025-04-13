use byteorder::{LittleEndian, ReadBytesExt};
use std::io::Cursor;
use std::io::Read;

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
    pub _padding: [u8; 11],
    pub crc32: u32,
    pub timestamp: u64,
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
