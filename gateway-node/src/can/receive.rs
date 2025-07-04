use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embedded_can::{Frame, Id};                                           
use esp_hal::twai::{filter::SingleStandardFilter, *};
use esp_println::println;
use heapless::spsc::Queue;
use heapless::String;
use core::fmt::Write; // để xài write! macro
use crc::{Crc, CRC_8_AUTOSAR};
use core::cell::RefCell;
use critical_section::Mutex;
use log::{debug, error, info};

pub static QUEUE: Mutex<RefCell<Queue<String<128>, 32>>> = Mutex::new(RefCell::new(Queue::new()));

#[derive(Debug)]
#[allow(dead_code)]
pub struct ImuData {
    temperature: f32,
    accelerometer: (f32, f32, f32),
    gyroscope: (f32, f32, f32),
}

impl ImuData {

    pub fn format(&self) -> String<128> {
        let mut s = String::<128>::new();

        write!(
            &mut s,
            "Temperature: {:.2} °C, Accelerometer: x={:.3}g y={:.3}g z={:.3}g, Gyroscope: x={:.3}°/s y={:.3}°/s z={:.3}°/s",
            self.temperature,
            self.accelerometer.0, self.accelerometer.1, self.accelerometer.2,
            self.gyroscope.0, self.gyroscope.1, self.gyroscope.2,
        ).unwrap();

        s
    }

    pub fn new(data: [u8;15]) -> Self {
        fn bytes_to_i16(msb: u8, lsb: u8) -> i16 {
            ((msb as i16) << 8) | (lsb as i16 & 0xFF)
        }

        let temp_raw = bytes_to_i16(data[1], data[0]);
        let temperature = (temp_raw as f32) / 340.0 + 36.53;

        // Accelerometer
        let accel_x = bytes_to_i16(data[9], data[8]) as f32;
        let accel_y = bytes_to_i16(data[11], data[10]) as f32;
        let accel_z = bytes_to_i16(data[13], data[12]) as f32;
        let accelerometer = (
            accel_x / 16384.0,
            accel_y / 16384.0,
            accel_z / 16384.0,
        );

        // Gyroscope
        let gyro_x = bytes_to_i16(data[3], data[2]) as f32;
        let gyro_y = bytes_to_i16(data[5], data[4]) as f32;
        let gyro_z = bytes_to_i16(data[7], data[6]) as f32;

        let gyroscope = (
            gyro_x / 131.0,
            gyro_y / 131.0,
            gyro_z / 131.0,
        );

        ImuData {temperature, accelerometer, gyroscope,}
    }
    
    pub fn enqueue_json_string(&self) {
        let (ax, ay, az) = self.accelerometer;
        let (gx, gy, gz) = self.gyroscope;

        let mut json_string: String<128> = String::new();

        write!(
            &mut json_string,
            "{{\"temperature\":{:.2},\"ax\":{:.2},\"ay\":{:.2},\"az\":{:.2},\"gx\":{:.2},\"gy\":{:.2},\"gz\":{:.2}}}",
            self.temperature, ax, ay, az, gx, gy, gz
        ).expect("Failed to format JSON string");

        critical_section::with(|cs| {
            match QUEUE.borrow_ref_mut(cs).enqueue(json_string) {
                Ok(()) => info!("Enqueued IMU data successfully"),
                Err(_) => error!("Failed to enqueue IMU data: queue full"),
            }
        });
    }

}

pub struct CANSetUp;
impl CANSetUp{
    pub fn new<'a>(mut can_config: TwaiConfiguration<'a, esp_hal::Async>) -> Twai<'a, esp_hal::Async> {
        //setup CAN1 filter:
        can_config.set_filter(const{
            SingleStandardFilter::new(b"xxxxxxxxxxx", b"x", [b"xxxxxxxx", b"xxxxxxxx"])
        });
        can_config.start()
    }
}


// pub type TwaiOutbox = Channel<NoopRawMutex, String<128>, 10>;


// const IMU_ID: Id::Standard = StandardId(0x120);


#[embassy_executor::task]
pub async fn can_receiver(
    mut rx: TwaiRx<'static, esp_hal::Async>) -> ! {

    let mut buffer = [0u8; 15];
    let mut received = [false; 3];
    loop {
        let frame = rx.receive_async().await;
        match frame {
            Ok(frame) => {
                // repeat the frame back
                match frame.id() {
                    // ION doesn't work with Extended
                    Id::Standard(id) => {   
                        // println!("Received a packet: ID {:?} - Type {:?}",frame.id(), frame.data());
                        if id.as_raw() == 0x120{
                            let data = frame.data();
                            let order = data[0];
                            insert_can_packet(order, data, &mut buffer, &mut received);
                            
                        }

                    }
                    Id::Extended(_id) => {}
                }
            }
            Err(e) => {error!("{e:?}")}
        }
        //check correct and send data
        if received.iter().all(|&r| r) {

            if true {
                let message = ImuData::new(buffer);

                message.enqueue_json_string();
            }
            received = [false; 3]; // reset

        }
    }
}


fn insert_can_packet(order: u8, data: &[u8], buffer: &mut [u8; 15], received: &mut [bool; 3]) -> bool{
    let payload = &data[1..]; // get payload
    // println!("order {} payload {:?}!!", order, payload);
    match order {
        0 => {
            // println!("Temperature!!");
            buffer[0..2].copy_from_slice(&payload[0..2]);
            received[0] = true;
            true
        }
        1 => {
            // println!("accelerometer!!");
            buffer[2..8].copy_from_slice(&payload[0..6]); 
            received[1] = true;
            true
        }
        2 => {
            // println!("gyroscope!!");
            buffer[8..15].copy_from_slice(&payload[0..7]); 
            received[2] = true;
            true
        }
        _ => false,
    }    
}


fn check_crc(packet: &[u8; 15]) -> bool {
    const X8: Crc<u8> = Crc::<u8>::new(&CRC_8_AUTOSAR);
    
    let crc_calculated = X8.checksum(&packet[0..14]);
    let crc_expected = packet[14];
    
    crc_calculated == crc_expected
}