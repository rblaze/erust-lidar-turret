use core::num::NonZeroU16;

use crate::error::Error;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DataPoint {
    Unreliable,
    Value(NonZeroU16),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Report {
    ZeroMark,
    Unreliable,
    Value(u16),
}

impl From<DataPoint> for Report {
    fn from(dp: DataPoint) -> Self {
        match dp {
            DataPoint::Unreliable => Report::Unreliable,
            DataPoint::Value(v) => Report::Value(v.get()),
        }
    }
}

#[derive(Debug)]
pub struct DistanceQueue {
    host_usart_reader: usize,
    lidar_reader: usize,
    writer: usize,
    queue: [DataPoint; Self::QUEUE_SIZE],
    message_position: usize,
    message: [u8; Self::MESSAGE_SIZE],
    host_usart_mark: Option<usize>,
    lidar_mark: Option<usize>,
}

impl DistanceQueue {
    const QUEUE_SIZE: usize = 128;
    const MESSAGE_SIZE: usize = 9;

    pub const fn new() -> Self {
        Self {
            host_usart_reader: 0,
            lidar_reader: 0,
            writer: 0,
            queue: [DataPoint::Unreliable; Self::QUEUE_SIZE],
            message_position: 0,
            message: [0; Self::MESSAGE_SIZE],
            host_usart_mark: None,
            lidar_mark: None,
        }
    }

    /// Append a byte to the queue. Returns true if a complete distance message was parsed.
    pub fn push_byte(&mut self, byte: u8) -> Result<bool, Error> {
        const LAST_POSITION: usize = DistanceQueue::MESSAGE_SIZE - 1;

        let distance_added = match self.message_position {
            0 => {
                if byte == 0x59 {
                    self.message[0] = byte;
                    self.message_position += 1;
                }
                false
            }
            1 => {
                if byte == 0x59 {
                    self.message[1] = byte;
                    self.message_position += 1;
                } else {
                    self.message_position = 0;
                }
                false
            }
            LAST_POSITION => {
                self.message[LAST_POSITION] = byte;
                // Message is now fully read.
                let amp = u16::from_le_bytes([self.message[4], self.message[5]]);
                let distance = u16::from_le_bytes([self.message[2], self.message[3]]);
                self.message_position = 0;
                if amp < 100 || amp == 0xffff {
                    self.append_datapoint(DataPoint::Unreliable)?;
                } else {
                    let value = NonZeroU16::new(distance)
                        .map(DataPoint::Value)
                        .unwrap_or(DataPoint::Unreliable);
                    self.append_datapoint(value)?;
                }
                true
            }
            i => {
                self.message[i] = byte;
                self.message_position += 1;
                false
            }
        };

        Ok(distance_added)
    }

    fn append_datapoint(&mut self, value: DataPoint) -> Result<(), Error> {
        let next_write_pos = (self.writer + 1) % Self::QUEUE_SIZE;
        if next_write_pos == self.host_usart_reader || next_write_pos == self.lidar_reader {
            return Err(Error::QueueOverrun);
        }

        self.queue[self.writer] = value;
        self.writer = next_write_pos;

        Ok(())
    }

    pub fn read_for_lidar(&mut self) -> Option<Report> {
        Self::read_from(
            &self.queue,
            self.writer,
            &mut self.lidar_reader,
            &mut self.lidar_mark,
        )
    }

    pub fn read_for_host_usart(&mut self) -> Option<Report> {
        Self::read_from(
            &self.queue,
            self.writer,
            &mut self.host_usart_reader,
            &mut self.host_usart_mark,
        )
    }

    fn read_from(
        queue: &[DataPoint; Self::QUEUE_SIZE],
        writer: usize,
        reader: &mut usize,
        mark: &mut Option<usize>,
    ) -> Option<Report> {
        if Some(*reader) == *mark {
            *mark = None;
            return Some(Report::ZeroMark);
        }

        if *reader == writer {
            return None;
        }

        let value = queue[*reader];
        *reader = (*reader + 1) % Self::QUEUE_SIZE;
        Some(Report::from(value))
    }

    pub fn set_zero_mark(&mut self) {
        self.host_usart_mark = Some(self.writer);
        self.lidar_mark = Some(self.writer);
    }
}

impl Default for DistanceQueue {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dp(value: u16) -> DataPoint {
        DataPoint::Value(NonZeroU16::new(value).unwrap())
    }

    #[test]
    fn test_new_queue_empty() {
        let mut dq = DistanceQueue::new();
        assert_eq!(dq.read_for_lidar(), None);
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_append_and_read() {
        let mut dq = DistanceQueue::new();
        dq.append_datapoint(dp(42)).unwrap();
        dq.append_datapoint(dp(100)).unwrap();

        assert_eq!(dq.read_for_lidar(), Some(Report::Value(42)));
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(100)));
        assert_eq!(dq.read_for_lidar(), None);

        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(42)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(100)));
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_queue_overrun() {
        let mut dq = DistanceQueue::new();
        // The queue has capacity of QUEUE_SIZE - 1 = 127 items.
        for i in 1..DistanceQueue::QUEUE_SIZE {
            dq.append_datapoint(dp(i as u16)).unwrap();
        }

        // The 128th append should fail with QueueOverrun.
        assert_eq!(dq.append_datapoint(dp(128)), Err(Error::QueueOverrun));

        // If we read one item from both readers, we should be able to append one more.
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(1)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(1)));

        dq.append_datapoint(dp(128)).unwrap();
        assert_eq!(dq.append_datapoint(dp(129)), Err(Error::QueueOverrun));
    }

    #[test]
    fn test_independent_readers() {
        let mut dq = DistanceQueue::new();
        dq.append_datapoint(dp(10)).unwrap();
        dq.append_datapoint(dp(20)).unwrap();

        // Read one from lidar
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(10)));

        // Append another one
        dq.append_datapoint(dp(30)).unwrap();

        // Read all from host usart
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(10)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(20)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(30)));
        assert_eq!(dq.read_for_host_usart(), None);

        // Read remainder from lidar
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(20)));
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(30)));
        assert_eq!(dq.read_for_lidar(), None);
    }

    #[test]
    fn test_valid_packet() {
        let mut dq = DistanceQueue::new();
        let packet: [u8; 9] = [0x59, 0x59, 0x34, 0x12, 0x05, 0x06, 0x07, 0x08, 0x09];

        // Send first 8 bytes
        for &b in &packet[..8] {
            assert_eq!(dq.push_byte(b), Ok(false));
            assert_eq!(dq.read_for_lidar(), None);
        }

        // Send the 9th byte
        assert_eq!(dq.push_byte(packet[8]), Ok(true));
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(0x1234)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(0x1234)));
    }

    #[test]
    fn test_partial_match_and_reset() {
        let mut dq = DistanceQueue::new();
        // Send 0x59, then non-0x59 (0x01) -> should reset position to 0
        assert_eq!(dq.push_byte(0x59), Ok(false));
        assert_eq!(dq.push_byte(0x01), Ok(false));

        // Now send a valid packet
        let packet: [u8; 9] = [0x59, 0x59, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
        for &b in &packet {
            dq.push_byte(b).unwrap();
        }
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(0x0201)));
    }

    #[test]
    fn test_payload_contains_header() {
        let mut dq = DistanceQueue::new();
        // Send a packet where all bytes are 0x59 (both header and payload)
        let packet: [u8; 9] = [0x59; 9];
        for &b in &packet {
            dq.push_byte(b).unwrap();
        }
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(0x5959)));

        // Send another normal packet to verify the parser works correctly afterwards
        let packet2: [u8; 9] = [0x59, 0x59, 0x11, 0x22, 100, 0, 0, 0, 0];
        for &b in &packet2 {
            dq.push_byte(b).unwrap();
        }
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(0x2211)));
    }

    #[test]
    fn test_queue_overrun_in_push_byte() {
        let mut dq = DistanceQueue::new();

        // Fill the queue to capacity (127 items)
        for i in 1..=127 {
            let low = (i & 0xFF) as u8;
            let high = ((i >> 8) & 0xFF) as u8;
            let packet: [u8; 9] = [0x59, 0x59, low, high, 100, 0, 0, 0, 0];
            for &b in &packet {
                dq.push_byte(b).unwrap();
            }
        }

        // The 128th packet should cause an overrun on its final byte
        let overrun_packet: [u8; 9] = [0x59, 0x59, 0x80, 0x00, 100, 0, 0, 0, 0];
        for &b in &overrun_packet[..8] {
            assert_eq!(dq.push_byte(b), Ok(false));
        }
        // The last byte tries to append to the full queue and should fail with QueueOverrun
        assert_eq!(dq.push_byte(overrun_packet[8]), Err(Error::QueueOverrun));

        // Read one item from both readers to free a slot
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(1)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(1)));

        // The slot is now consumed. Check if the distance was appended.
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(2))); // First slot was consumed, next is 2
        // Skip ahead to the end of the queue
        for _ in 0..125 {
            dq.read_for_lidar().unwrap();
        }
        assert_eq!(dq.read_for_lidar(), None);
    }

    #[test]
    fn test_set_mark_empty_queue() {
        let mut dq = DistanceQueue::new();
        // Set mark on empty queue (writer at 0)
        dq.set_zero_mark();

        // Reading should immediately trigger the mark and return ZeroMark for both readers
        assert_eq!(dq.read_for_lidar(), Some(Report::ZeroMark));
        assert_eq!(dq.read_for_lidar(), None);

        assert_eq!(dq.read_for_host_usart(), Some(Report::ZeroMark));
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_set_mark_with_data() {
        let mut dq = DistanceQueue::new();
        dq.append_datapoint(dp(10)).unwrap();
        dq.append_datapoint(dp(20)).unwrap();

        // Set mark. Writer is currently at 2.
        dq.set_zero_mark();

        // Lidar reader should read 10 and 20 first, then get ZeroMark, then None
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(10)));
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(20)));
        assert_eq!(dq.read_for_lidar(), Some(Report::ZeroMark));
        assert_eq!(dq.read_for_lidar(), None);

        // Host usart reader should read 10 and 20 first, then get ZeroMark, then None
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(10)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(20)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::ZeroMark));
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_set_mark_then_append_more() {
        let mut dq = DistanceQueue::new();
        dq.append_datapoint(dp(10)).unwrap();

        // Set mark. Writer is currently at 1.
        dq.set_zero_mark();

        // Append more data after setting the mark
        dq.append_datapoint(dp(20)).unwrap();
        dq.append_datapoint(dp(30)).unwrap();

        // Lidar reader should read:
        // 1. 10 (reader was at 0, mark is at 1)
        // 2. ZeroMark (reader is at 1, matches mark at 1)
        // 3. 20 (reader continues at 1)
        // 4. 30 (reader is at 2)
        // 5. None (reader is at 3, equal to writer)
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(10)));
        assert_eq!(dq.read_for_lidar(), Some(Report::ZeroMark));
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(20)));
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(30)));
        assert_eq!(dq.read_for_lidar(), None);

        // Host usart reader should read the same sequence
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(10)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::ZeroMark));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(20)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(30)));
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_set_mark_multiple_times() {
        let mut dq = DistanceQueue::new();
        dq.append_datapoint(dp(10)).unwrap();

        // First mark at 1
        dq.set_zero_mark();

        dq.append_datapoint(dp(20)).unwrap();

        // Second mark overwrites the first one, now at 2
        dq.set_zero_mark();

        dq.append_datapoint(dp(30)).unwrap();

        // Lidar reader
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(10)));
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(20)));
        assert_eq!(dq.read_for_lidar(), Some(Report::ZeroMark));
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(30)));
        assert_eq!(dq.read_for_lidar(), None);

        // Host usart reader
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(10)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(20)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::ZeroMark));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(30)));
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_mark_consumed_independently() {
        let mut dq = DistanceQueue::new();
        dq.append_datapoint(dp(10)).unwrap();
        dq.set_zero_mark();
        dq.append_datapoint(dp(20)).unwrap();

        // Lidar reader consumes its mark first
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(10)));
        assert_eq!(dq.read_for_lidar(), Some(Report::ZeroMark));
        assert_eq!(dq.read_for_lidar(), Some(Report::Value(20)));
        assert_eq!(dq.read_for_lidar(), None);

        // Host usart reader mark was unaffected and can still be consumed independently
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(10)));
        assert_eq!(dq.read_for_host_usart(), Some(Report::ZeroMark));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Value(20)));
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_unreliable_datapoint() {
        let mut dq = DistanceQueue::new();
        // Low amplitude (< 100) -> Unreliable
        let low_amp_packet: [u8; 9] = [0x59, 0x59, 0x10, 0x00, 99, 0, 0, 0, 0];
        for &b in &low_amp_packet {
            dq.push_byte(b).unwrap();
        }
        assert_eq!(dq.read_for_lidar(), Some(Report::Unreliable));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Unreliable));

        // Amplitude == 0xffff -> Unreliable
        let max_amp_packet: [u8; 9] = [0x59, 0x59, 0x10, 0x00, 0xff, 0xff, 0, 0, 0];
        for &b in &max_amp_packet {
            dq.push_byte(b).unwrap();
        }
        assert_eq!(dq.read_for_lidar(), Some(Report::Unreliable));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Unreliable));

        // Zero distance -> Unreliable
        let zero_dist_packet: [u8; 9] = [0x59, 0x59, 0x00, 0x00, 100, 0, 0, 0, 0];
        for &b in &zero_dist_packet {
            dq.push_byte(b).unwrap();
        }
        assert_eq!(dq.read_for_lidar(), Some(Report::Unreliable));
        assert_eq!(dq.read_for_host_usart(), Some(Report::Unreliable));
    }
}
