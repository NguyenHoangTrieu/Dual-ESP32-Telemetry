use defmt::println;
use nb;
use esp_hal::{
    twai::{filter::SingleStandardFilter, *},
    delay::Delay,
};
// use esp_println::println;
use esp_backtrace as _;
use esp_println::{self as _};
use esp_hal::twai::ErrorKind::Other;
// use esp_hal::delay::Delay;
pub struct CANTransfer;
impl CANTransfer{
    pub fn new<'a>(mut can_config: TwaiConfiguration<'a, esp_hal::Blocking>) -> Twai<'a, esp_hal::Blocking> {
        //setup CAN1 filter:
        can_config.set_filter(const{
            SingleStandardFilter::new(b"xxxxxxxxxxx", b"x", [b"xxxxxxxx", b"xxxxxxxx"])
        });
        can_config.start()
    }
    // Function to send 15-byte sensor data in three CAN frames

    pub fn send_sensor_data(can: &mut Twai<'_, esp_hal::Blocking>, data: &[u8; 15]) -> Result<(), EspTwaiError> {
        // Define frame IDs for the three chunks
        const TEMPERATURE: u8 = 0x00;
        const ACCELEROMETER: u8 = 0x01;
        const GYROSCOPE: u8 = 0x02;
        const FRAME_ID: u16 = 0x120;  // Chunk ID

        let mut chunk1 = [0u8; 3];
        chunk1[0] = TEMPERATURE;
        chunk1[1..].copy_from_slice(&data[0..2]);  // Bytes 0 - 1
        
        let mut chunk2 = [0u8; 7];
        chunk2[0] = ACCELEROMETER;
        chunk2[1..].copy_from_slice(&data[2..8]);  // Bytes 2 - 8

        let mut chunk3 = [0u8; 8];
        chunk3[0] = GYROSCOPE;
        chunk3[1..].copy_from_slice(&data[8..15]);  // Bytes 8 - 15

        let delay = Delay::new();

        defmt::println!("Transmit Error: {:?}", can.transmit_error_count());

        // Send first chunk
        let frame1 = EspTwaiFrame::new(StandardId::new(FRAME_ID).unwrap(), &chunk1)
            .ok_or(EspTwaiError::EmbeddedHAL(Other))?;
        match can.transmit(&frame1) {
            Ok(()) => defmt::println!("Frame 1 sent: {:?}", chunk1),
            Err(nb::Error::WouldBlock) => {
                defmt::println!("Frame 1 would block, retrying...");
                // retry logic if needed ?
                return Err(EspTwaiError::EmbeddedHAL(Other));
            },
            Err(nb::Error::Other(e)) => {
                defmt::error!("Frame 1 error: {:?}", e);
                return Err(e);
            }
        }
        delay.delay_micros(300);
        // Send second chunk
        let frame2 = EspTwaiFrame::new(StandardId::new(FRAME_ID).unwrap(), &chunk2)
        .ok_or(EspTwaiError::EmbeddedHAL(Other))?;
    
        match can.transmit(&frame2) {
            Ok(()) => defmt::println!("Frame 2 sent: {:?}", chunk2),
            Err(nb::Error::WouldBlock) => {
                defmt::error!("Frame 2 would block, retrying...");
                return Err(EspTwaiError::EmbeddedHAL(Other));
            },
            Err(nb::Error::Other(e)) => {
                defmt::error!("Frame 2 error: {:?}", e);
                return Err(e);
            }
        }
        delay.delay_micros(300);
        // Send third chunk
        let frame3 = EspTwaiFrame::new(StandardId::new(FRAME_ID).unwrap(), &chunk3)
        .ok_or(EspTwaiError::EmbeddedHAL(Other))?;
    
        match can.transmit(&frame3) {
            Ok(()) => defmt::println!("Frame 3 sent: {:?}", chunk3),
            Err(nb::Error::WouldBlock) => {
                defmt::error!("Frame 3 would block, retrying...");
                return Err(EspTwaiError::EmbeddedHAL(Other));
            },
            Err(nb::Error::Other(e)) => {
                defmt::error!("Frame 3 error: {:?}", e);
                return Err(e);
            }
        }
        Ok(())
    }

    
}