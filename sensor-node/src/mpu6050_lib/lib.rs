#![no_std]

use crate::mpu6050_lib::bits;
use super::mpu::*;
use libm::{powf, atan2f, sqrtf};
use micromath::vector::{Vector3d, Vector2d};
use embedded_hal::{
    delay::DelayNs,
    i2c::{Error, ErrorType, ErrorKind, I2c, SevenBitAddress},
};
/// PI, f32
pub const PI: f32 = core::f32::consts::PI;

/// PI / 180, for conversion to radians
pub const PI_180: f32 = PI / 180.0;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Mpu6050Error<E> {
    /// I2C bus error
    I2c(E),

    /// Invalid chip ID was read
    InvalidChipId(u8),
}

/// Handles all operations on/with Mpu6050
pub struct Mpu6050<I> {
    i2c: I,
    slave_addr: u8,
    acc_sensitivity: f32,
    gyro_sensitivity: f32,
}
#[allow(dead_code)]
impl<I> Mpu6050<I>
where
    I: I2c<SevenBitAddress>,
{
    /// Side effect free constructor with default sensitivies, no calibration
    pub fn new(i2c: I) -> Self {
        Mpu6050 {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            acc_sensitivity: ACCEL_SENS.0,
            gyro_sensitivity: GYRO_SENS.0,
        }
    }

    /// custom sensitivity
    pub fn new_with_sens(i2c: I, arange: AccelRange, grange: GyroRange) -> Self {
        Mpu6050 {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            acc_sensitivity: arange.sensitivity(),
            gyro_sensitivity: grange.sensitivity(),
        }
    }

    /// Same as `new`, but the chip address can be specified (e.g. 0x69, if the A0 pin is pulled up)
    pub fn new_with_addr(i2c: I, slave_addr: u8) -> Self {
        Mpu6050 {
            i2c,
            slave_addr,
            acc_sensitivity: ACCEL_SENS.0,
            gyro_sensitivity: GYRO_SENS.0,
        }
    }

    /// Combination of `new_with_sens` and `new_with_addr`
    pub fn new_with_addr_and_sens(i2c: I, slave_addr: u8, arange: AccelRange, grange: GyroRange) -> Self {
        Mpu6050 {
            i2c,
            slave_addr,
            acc_sensitivity: arange.sensitivity(),
            gyro_sensitivity: grange.sensitivity(),
        }
    }

    /// Wakes MPU6050 with all sensors enabled (default)
    fn wake<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), I::Error> {
        // MPU6050 has sleep enabled by default -> set bit 0 to wake
        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 (See Register Map )
        self.write_byte(PWR_MGMT_1::ADDR, 0x01)?;
        delay.delay_ms(100u32);
        Ok(())
    }

    /// From Register map:
    /// "An  internal  8MHz  oscillator,  gyroscope based  clock,or  external  sources  can  be
    /// selected  as the MPU-60X0 clock source.
    /// When the internal 8 MHz oscillator or an external source is chosen as the clock source,
    /// the MPU-60X0 can operate in low power modes with the gyroscopes disabled. Upon power up,
    /// the MPU-60X0clock source defaults to the internal oscillator. However, it is highly
    /// recommended  that  the  device beconfigured  to  use  one  of  the  gyroscopes
    /// (or  an  external  clocksource) as the clock reference for improved stability.
    /// The clock source can be selected according to the following table...."
    pub fn set_clock_source(&mut self, source: CLKSEL) -> Result<(), I::Error> {
        Ok(self.write_bits(PWR_MGMT_1::ADDR, PWR_MGMT_1::CLKSEL.bit, PWR_MGMT_1::CLKSEL.length, source as u8)?)
    }

    /// get current clock source
    pub fn get_clock_source(&mut self) -> Result<CLKSEL, I::Error> {
        let source = self.read_bits(PWR_MGMT_1::ADDR, PWR_MGMT_1::CLKSEL.bit, PWR_MGMT_1::CLKSEL.length)?;
        Ok(CLKSEL::from(source))
    }

    /// Init wakes MPU6050 and verifies register addr, e.g. in i2c
    pub fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), I::Error> {
        self.wake(delay)?;
        // self.verify().unwrap();
        self.set_accel_range(AccelRange::G2)?;
        self.set_gyro_range(GyroRange::D250)?;
        self.set_accel_hpf(ACCEL_HPF::_RESET)?;
        self.set_congfig(DlpfConfig::Hz94)?;
        Ok(())
    }

    /// Verifies device to address 0x68 with WHOAMI.addr() Register
	fn verify(&mut self) -> Result<(), Mpu6050Error<I::Error>> {
		let address = self.read_byte(WHOAMI).map_err(Mpu6050Error::I2c)?;
		if address != 0x70 {
			return Err(Mpu6050Error::InvalidChipId(address));
		}
		Ok(())
	}

    /// setup motion detection
    pub fn setup_motion_detection(&mut self) -> Result<(), I::Error> {
        self.write_byte(0x6B, 0x00)?;
        // optional? self.write_byte(0x68, 0x07)?; // Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
        self.write_byte(INT_PIN_CFG::ADDR, 0x20)?; //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
        self.write_byte(ACCEL_CONFIG::ADDR, 0x01)?; //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
        self.write_byte(MOT_THR, 10)?; //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).
        self.write_byte(MOT_DUR, 40)?; //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
        self.write_byte(0x69, 0x15)?; //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )
        self.write_byte(INT_ENABLE::ADDR, 0x40)?; //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.
        Ok(())
    }

    pub fn set_congfig(&mut self, val: DlpfConfig) -> Result<(), I::Error> {
        Ok(
            self.write_bits(CONFIG::ADDR,
                            CONFIG::DLPF_CFG.bit,
                            CONFIG::DLPF_CFG.length,
                            val as u8)?
        )
    }

    /// get whether or not motion has been detected (INT_STATUS, MOT_INT)
    pub fn get_motion_detected(&mut self) -> Result<bool, I::Error> {
        Ok(self.read_bit(INT_STATUS::ADDR, INT_STATUS::MOT_INT)? != 0)
    }

    /// set accel high pass filter mode
    pub fn set_accel_hpf(&mut self, mode: ACCEL_HPF) -> Result<(), I::Error> {
        Ok(
            self.write_bits(ACCEL_CONFIG::ADDR,
                            ACCEL_CONFIG::ACCEL_HPF.bit,
                            ACCEL_CONFIG::ACCEL_HPF.length,
                            mode as u8)?
        )
    }

    /// get accel high pass filter mode
    pub fn get_accel_hpf(&mut self) -> Result<ACCEL_HPF, I::Error> {
        let mode: u8 = self.read_bits(ACCEL_CONFIG::ADDR,
                                      ACCEL_CONFIG::ACCEL_HPF.bit,
                                      ACCEL_CONFIG::ACCEL_HPF.length)?;

        Ok(ACCEL_HPF::from(mode))
    }

    /// Set gyro range, and update sensitivity accordingly
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), I::Error> {
        self.write_bits(GYRO_CONFIG::ADDR,
                        GYRO_CONFIG::FS_SEL.bit,
                        GYRO_CONFIG::FS_SEL.length,
                        range as u8)?;

        self.gyro_sensitivity = range.sensitivity();
        Ok(())
    }

    /// get current gyro range
    pub fn get_gyro_range(&mut self) -> Result<GyroRange, I::Error> {
        let byte = self.read_bits(GYRO_CONFIG::ADDR,
                                  GYRO_CONFIG::FS_SEL.bit,
                                  GYRO_CONFIG::FS_SEL.length)?;

        Ok(GyroRange::from(byte))
    }

    /// set accel range, and update sensitivy accordingly
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), I::Error> {
        self.write_bits(ACCEL_CONFIG::ADDR,
                        ACCEL_CONFIG::FS_SEL.bit,
                        ACCEL_CONFIG::FS_SEL.length,
                        range as u8)?;

        self.acc_sensitivity = range.sensitivity();
        Ok(())
    }

    /// get current accel_range
    pub fn get_accel_range(&mut self) -> Result<AccelRange, I::Error> {
        let byte = self.read_bits(ACCEL_CONFIG::ADDR,
                                  ACCEL_CONFIG::FS_SEL.bit,
                                  ACCEL_CONFIG::FS_SEL.length)?;

        Ok(AccelRange::from(byte))
    }

    /// reset device
    pub fn reset_device<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), I::Error> {
        self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::DEVICE_RESET, true)?;
        delay.delay_ms(100u32);
        // Note: Reset sets sleep to true! Section register map: resets PWR_MGMT to 0x40
        Ok(())
    }

    /// enable, disable sleep of sensor
    pub fn set_sleep_enabled(&mut self, enable: bool) -> Result<(), I::Error> {
        Ok(self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP, enable)?)
    }

    /// get sleep status
    pub fn get_sleep_enabled(&mut self) -> Result<bool, I::Error> {
        Ok(self.read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP)? != 0)
    }

    /// enable, disable temperature measurement of sensor
    /// TEMP_DIS actually saves "disabled status"
    /// 1 is disabled! -> enable=true : bit=!enable
    pub fn set_temp_enabled(&mut self, enable: bool) -> Result<(), I::Error> {
        Ok(self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS, !enable)?)
    }

    /// get temperature sensor status
    /// TEMP_DIS actually saves "disabled status"
    /// 1 is disabled! -> 1 == 0 : false, 0 == 0 : true
    pub fn get_temp_enabled(&mut self) -> Result<bool, I::Error> {
        Ok(self.read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS)? == 0)
    }

    /// set accel x self test
    pub fn set_accel_x_self_test(&mut self, enable: bool) -> Result<(), I::Error> {
        Ok(self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST, enable)?)
    }

    /// get accel x self test
    pub fn get_accel_x_self_test(&mut self) -> Result<bool, I::Error> {
        Ok(self.read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST)? != 0)
    }

    /// set accel y self test
    pub fn set_accel_y_self_test(&mut self, enable: bool) -> Result<(), I::Error> {
        Ok(self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST, enable)?)
    }

    /// get accel y self test
    pub fn get_accel_y_self_test(&mut self) -> Result<bool, I::Error> {
        Ok(self.read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST)? != 0)
    }

    /// set accel z self test
    pub fn set_accel_z_self_test(&mut self, enable: bool) -> Result<(), I::Error> {
        Ok(self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST, enable)?)
    }

    /// get accel z self test
    pub fn get_accel_z_self_test(&mut self) -> Result<bool, I::Error> {
        Ok(self.read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST)? != 0)
    }

    /// Roll and pitch estimation from raw accelerometer readings
    /// NOTE: no yaw! no magnetometer present on MPU6050
    /// https://www.nxp.com/docs/en/application-note/AN3461.pdf equation 28, 29
    pub fn get_acc_angles(&mut self) -> Result<Vector2d<f32>, I::Error> {
        let acc = self.get_acc()?;

        Ok(Vector2d::<f32>{
            x: atan2f(acc.y, sqrtf(powf(acc.x, 2.) + powf(acc.z, 2.))),
            y: atan2f(-acc.x, sqrtf(powf(acc.y, 2.) + powf(acc.z, 2.)))
	})
    }

    /// Converts 2 bytes number in 2 compliment
    /// TODO i16?! whats 0x8000?!
    fn read_word_2c(&self, byte: &[u8]) -> i32 {
        let high: i32 = byte[0] as i32;
        let low: i32 = byte[1] as i32;
        let mut word: i32 = (high << 8) + low;

        if word >= 0x8000 {
            word = -((65535 - word) + 1);
        }

        word
    }

    /// Reads rotation (gyro/acc) from specified register
    fn read_rot(&mut self, reg: u8) -> Result<Vector3d<f32>, I::Error> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf)?;

        Ok(Vector3d::<f32>{
            x: self.read_word_2c(&buf[0..2]) as f32,
            y: self.read_word_2c(&buf[2..4]) as f32,
            z: self.read_word_2c(&buf[4..6]) as f32
		})
    }

    pub fn read_rot_raw_i16(&mut self, reg: u8) -> Result<Vector3d<i16>, I::Error> {
        let mut buf = [0u8; 6];
        self.read_bytes(reg, &mut buf)?;

        Ok(Vector3d::<i16> {
            x: self.read_word_2c(&buf[0..2]) as i16,
            y: self.read_word_2c(&buf[2..4]) as i16,
            z: self.read_word_2c(&buf[4..6]) as i16,
        })
    }

    /// Accelerometer readings in g
    pub fn get_acc(&mut self) -> Result<Vector3d<f32>, I::Error> {
        let mut acc = self.read_rot(ACC_REGX_H)?;
        acc = Vector3d {
        x: acc.x / self.acc_sensitivity,
        y: acc.y / self.acc_sensitivity,
        z: acc.z / self.acc_sensitivity,
    	};


        Ok(acc)
    }

    /// Gyro readings in rad/s
    pub fn get_gyro(&mut self) -> Result<Vector3d<f32>, I::Error> {
        let mut gyro = self.read_rot(GYRO_REGX_H)?;

        gyro *= PI_180 / self.gyro_sensitivity;

        Ok(gyro)
    }

    /// Sensor Temp in degrees celcius
    pub fn get_temp(&mut self) -> Result<f32, I::Error> {
        let mut buf: [u8; 2] = [0; 2];
        self.read_bytes(TEMP_OUT_H, &mut buf)?;
        let raw_temp = self.read_word_2c(&buf[0..2]) as f32;

        // According to revision 4.2
        Ok((raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET)
    }
    /// Get raw gyroscope readings (i16, unscaled)
    pub fn get_gyro_raw(&mut self) -> Result<Vector3d<i16>, I::Error> {
        self.read_rot_raw_i16(GYRO_REGX_H)
    }
    /// Get raw accelerometer readings (i16, unscaled)
    pub fn get_acc_raw(&mut self) -> Result<Vector3d<i16>, I::Error> {
        self.read_rot_raw_i16(ACC_REGX_H)
    }
    /// Get raw temperature from MPU6050 (i16, unscaled)
    pub fn get_temp_raw(&mut self) -> Result<i16, I::Error> {
        let mut buf = [0u8; 2];
        self.read_bytes(TEMP_OUT_H, &mut buf)?;
        Ok(self.read_word_2c(&buf) as i16)
    }

    /// Writes byte to register
    pub fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), I::Error> {
        self.i2c.write(self.slave_addr, &[reg, byte])?;
        // delay disabled for dev build
        // TODO: check effects with physical unit
        // self.delay.delay_ms(10u8);
        Ok(())
    }

    /// Enables bit n at register address reg
    pub fn write_bit(&mut self, reg: u8, bit_n: u8, enable: bool) -> Result<(), I::Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bit(&mut byte[0], bit_n, enable);
        Ok(self.write_byte(reg, byte[0])?)
    }

    /// Write bits data at reg from start_bit to start_bit+length
    pub fn write_bits(&mut self, reg: u8, start_bit: u8, length: u8, data: u8) -> Result<(), I::Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bits(&mut byte[0], start_bit, length, data);
        Ok(self.write_byte(reg, byte[0])?)
    }

    /// Read bit n from register
    fn read_bit(&mut self, reg: u8, bit_n: u8) -> Result<u8, I::Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        Ok(bits::get_bit(byte[0], bit_n))
    }

    /// Read bits at register reg, starting with bit start_bit, until start_bit+length
    pub fn read_bits(&mut self, reg: u8, start_bit: u8, length: u8) -> Result<u8, I::Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        Ok(bits::get_bits(byte[0], start_bit, length))
    }

    /// Reads byte from register
    pub fn read_byte(&mut self, reg: u8) -> Result<u8, I::Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c.write_read(self.slave_addr, &[reg], &mut byte)?;
        Ok(byte[0])
    }

    /// Reads series of bytes into buf from specified reg
    pub fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), I::Error> {
        self.i2c.write_read(self.slave_addr, &[reg], buf)?;
        Ok(())
    }
}