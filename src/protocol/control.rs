use crc::{Crc, CRC_16_IBM_3740, CRC_32_ISO_HDLC};
use std::sync::atomic::{AtomicU32, Ordering};

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
            std::mem::size_of_val(&header) - 6,
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
