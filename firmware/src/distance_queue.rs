use crate::error::Error;

pub struct DistanceQueue {
    host_usart_reader: usize,
    lidar_reader: usize,
    writer: usize,
    queue: [u16; Self::QUEUE_SIZE],
    message_position: usize,
    message: [u8; Self::MESSAGE_SIZE],
    host_usart_mark: Option<usize>,
}

impl DistanceQueue {
    const QUEUE_SIZE: usize = 128;
    const MESSAGE_SIZE: usize = 9;

    pub const fn new() -> Self {
        Self {
            host_usart_reader: 0,
            lidar_reader: 0,
            writer: 0,
            queue: [0; _],
            message_position: 0,
            message: [0; _],
            host_usart_mark: None,
        }
    }

    pub fn push_byte(&mut self, byte: u8) -> Result<bool, Error> {
        const LAST_POSITION: usize = DistanceQueue::MESSAGE_SIZE - 1;

        let distance_added = match self.message_position {
            0 => {
                if byte == 0x59 {
                    self.message_position += 1;
                }
                false
            }
            1 => {
                if byte == 0x59 {
                    self.message_position += 1;
                } else {
                    self.message_position = 0;
                }
                false
            }
            LAST_POSITION => {
                self.message[LAST_POSITION] = byte;
                // Message is now fully read.
                let distance = u16::from_le_bytes([self.message[2], self.message[3]]);
                self.message_position = 0;
                self.append_distance(distance)?;
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

    fn append_distance(&mut self, value: u16) -> Result<(), Error> {
        let next_write_pos = (self.writer + 1) % Self::QUEUE_SIZE;
        if next_write_pos == self.host_usart_reader || next_write_pos == self.lidar_reader {
            return Err(Error::QueueOverrun);
        }

        self.queue[self.writer] = value;
        self.writer = next_write_pos;

        Ok(())
    }

    pub fn read_for_lidar(&mut self) -> Option<u16> {
        if self.lidar_reader == self.writer {
            return None;
        }

        let value = self.queue[self.lidar_reader];
        self.lidar_reader = (self.lidar_reader + 1) % Self::QUEUE_SIZE;
        Some(value)
    }

    pub fn read_for_host_usart(&mut self) -> Option<u16> {
        if Some(self.host_usart_reader) == self.host_usart_mark {
            self.host_usart_mark = None;
            return Some(0xffff);
        }

        if self.host_usart_reader == self.writer {
            return None;
        }

        let value = self.queue[self.host_usart_reader];
        self.host_usart_reader = (self.host_usart_reader + 1) % Self::QUEUE_SIZE;
        Some(value)
    }

    pub fn set_mark_for_host_usart(&mut self) {
        self.host_usart_mark = Some(self.writer)
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

    #[test]
    fn test_new_queue_empty() {
        let mut dq = DistanceQueue::new();
        assert_eq!(dq.read_for_lidar(), None);
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_append_and_read() {
        let mut dq = DistanceQueue::new();
        dq.append_distance(42).unwrap();
        dq.append_distance(100).unwrap();

        assert_eq!(dq.read_for_lidar(), Some(42));
        assert_eq!(dq.read_for_lidar(), Some(100));
        assert_eq!(dq.read_for_lidar(), None);

        assert_eq!(dq.read_for_host_usart(), Some(42));
        assert_eq!(dq.read_for_host_usart(), Some(100));
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_queue_overrun() {
        let mut dq = DistanceQueue::new();
        // The queue has capacity of QUEUE_SIZE - 1 = 127 items.
        for i in 0..(DistanceQueue::QUEUE_SIZE - 1) {
            dq.append_distance(i as u16).unwrap();
        }

        // The 128th append should fail with QueueOverrun.
        assert_eq!(dq.append_distance(128), Err(Error::QueueOverrun));

        // If we read one item from both readers, we should be able to append one more.
        assert_eq!(dq.read_for_lidar(), Some(0));
        assert_eq!(dq.read_for_host_usart(), Some(0));

        dq.append_distance(128).unwrap();
        assert_eq!(dq.append_distance(129), Err(Error::QueueOverrun));
    }

    #[test]
    fn test_independent_readers() {
        let mut dq = DistanceQueue::new();
        dq.append_distance(10).unwrap();
        dq.append_distance(20).unwrap();

        // Read one from lidar
        assert_eq!(dq.read_for_lidar(), Some(10));

        // Append another one
        dq.append_distance(30).unwrap();

        // Read all from host usart
        assert_eq!(dq.read_for_host_usart(), Some(10));
        assert_eq!(dq.read_for_host_usart(), Some(20));
        assert_eq!(dq.read_for_host_usart(), Some(30));
        assert_eq!(dq.read_for_host_usart(), None);

        // Read remainder from lidar
        assert_eq!(dq.read_for_lidar(), Some(20));
        assert_eq!(dq.read_for_lidar(), Some(30));
        assert_eq!(dq.read_for_lidar(), None);
    }

    #[test]
    fn test_valid_packet() {
        let mut dq = DistanceQueue::new();
        let packet: [u8; 9] = [0x59, 0x59, 0x34, 0x12, 0x05, 0x06, 0x07, 0x08, 0x09];

        // Send first 8 bytes
        for i in 0..8 {
            assert_eq!(dq.push_byte(packet[i]), Ok(false));
            assert_eq!(dq.read_for_lidar(), None);
        }

        // Send the 9th byte
        assert_eq!(dq.push_byte(packet[8]), Ok(true));
        assert_eq!(dq.read_for_lidar(), Some(0x1234));
        assert_eq!(dq.read_for_host_usart(), Some(0x1234));
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
        assert_eq!(dq.read_for_lidar(), Some(0x0201));
    }

    #[test]
    fn test_payload_contains_header() {
        let mut dq = DistanceQueue::new();
        // Send a packet where all bytes are 0x59 (both header and payload)
        let packet: [u8; 9] = [0x59; 9];
        for &b in &packet {
            dq.push_byte(b).unwrap();
        }
        assert_eq!(dq.read_for_lidar(), Some(0x5959));

        // Send another normal packet to verify the parser works correctly afterwards
        let packet2: [u8; 9] = [0x59, 0x59, 0x11, 0x22, 0, 0, 0, 0, 0];
        for &b in &packet2 {
            dq.push_byte(b).unwrap();
        }
        assert_eq!(dq.read_for_lidar(), Some(0x2211));
    }

    #[test]
    fn test_queue_overrun_in_push_byte() {
        let mut dq = DistanceQueue::new();

        // Fill the queue to capacity (127 items)
        for i in 0..127 {
            let low = (i & 0xFF) as u8;
            let high = ((i >> 8) & 0xFF) as u8;
            let packet: [u8; 9] = [0x59, 0x59, low, high, 0, 0, 0, 0, 0];
            for &b in &packet {
                dq.push_byte(b).unwrap();
            }
        }

        // The 128th packet should cause an overrun on its final byte
        let overrun_packet: [u8; 9] = [0x59, 0x59, 0x80, 0x00, 0, 0, 0, 0, 0];
        for i in 0..8 {
            assert_eq!(dq.push_byte(overrun_packet[i]), Ok(false));
        }
        // The last byte tries to append to the full queue and should fail with QueueOverrun
        assert_eq!(dq.push_byte(overrun_packet[8]), Err(Error::QueueOverrun));

        // Read one item from both readers to free a slot
        assert_eq!(dq.read_for_lidar(), Some(0));
        assert_eq!(dq.read_for_host_usart(), Some(0));

        // The slot is now filled. Check if the distance was appended.
        assert_eq!(dq.read_for_lidar(), Some(1)); // First slot was consumed, next is 1
        // Skip ahead to the end of the queue to find the overrun packet's value (0x0080 = 128)
        for _ in 0..125 {
            dq.read_for_lidar().unwrap();
        }
        assert_eq!(dq.read_for_lidar(), None);
    }

    #[test]
    fn test_set_mark_empty_queue() {
        let mut dq = DistanceQueue::new();
        // Set mark on empty queue (writer at 0)
        dq.set_mark_for_host_usart();

        // Reading should immediately trigger the mark and return 0xffff
        assert_eq!(dq.read_for_host_usart(), Some(0xffff));
        // Next read should be empty since queue is empty
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_set_mark_with_data() {
        let mut dq = DistanceQueue::new();
        dq.append_distance(10).unwrap();
        dq.append_distance(20).unwrap();

        // Set mark. Writer is currently at 2.
        dq.set_mark_for_host_usart();

        // Reader should read 10 and 20 first, then get 0xffff, then None
        assert_eq!(dq.read_for_host_usart(), Some(10));
        assert_eq!(dq.read_for_host_usart(), Some(20));
        assert_eq!(dq.read_for_host_usart(), Some(0xffff));
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_set_mark_then_append_more() {
        let mut dq = DistanceQueue::new();
        dq.append_distance(10).unwrap();

        // Set mark. Writer is currently at 1.
        dq.set_mark_for_host_usart();

        // Append more data after setting the mark
        dq.append_distance(20).unwrap();
        dq.append_distance(30).unwrap();

        // Reader should read:
        // 1. 10 (reader was at 0, mark is at 1)
        // 2. 0xffff (reader is at 1, matches mark at 1)
        // 3. 20 (reader continues at 1)
        // 4. 30 (reader is at 2)
        // 5. None (reader is at 3, equal to writer)
        assert_eq!(dq.read_for_host_usart(), Some(10));
        assert_eq!(dq.read_for_host_usart(), Some(0xffff));
        assert_eq!(dq.read_for_host_usart(), Some(20));
        assert_eq!(dq.read_for_host_usart(), Some(30));
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_set_mark_multiple_times() {
        let mut dq = DistanceQueue::new();
        dq.append_distance(10).unwrap();

        // First mark at 1
        dq.set_mark_for_host_usart();

        dq.append_distance(20).unwrap();

        // Second mark overwrites the first one, now at 2
        dq.set_mark_for_host_usart();

        dq.append_distance(30).unwrap();

        // Reader should read:
        // 1. 10
        // 2. 20
        // 3. 0xffff (at index 2, because second mark was set when writer was at 2)
        // 4. 30
        // 5. None
        assert_eq!(dq.read_for_host_usart(), Some(10));
        assert_eq!(dq.read_for_host_usart(), Some(20));
        assert_eq!(dq.read_for_host_usart(), Some(0xffff));
        assert_eq!(dq.read_for_host_usart(), Some(30));
        assert_eq!(dq.read_for_host_usart(), None);
    }

    #[test]
    fn test_set_mark_does_not_affect_lidar() {
        let mut dq = DistanceQueue::new();
        dq.append_distance(10).unwrap();
        dq.set_mark_for_host_usart();
        dq.append_distance(20).unwrap();

        // Lidar reader should read 10, 20, then None (no 0xffff)
        assert_eq!(dq.read_for_lidar(), Some(10));
        assert_eq!(dq.read_for_lidar(), Some(20));
        assert_eq!(dq.read_for_lidar(), None);
    }
}
