#![no_std]
#![no_main]
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;  
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::time::Rate;
use esp_hal::timer::timg::{TimerGroup};
use esp_hal::twai::*;
use esp_hal::i2c::master::*;
use esp_hal::{Blocking};
use esp_backtrace as _;
use esp_println as _;
use micromath::vector::{Vector3d};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use core::cell::RefCell;
use critical_section::Mutex;
use sensor_node::mpu6050_lib::lib::Mpu6050;
use sensor_node::common_protocol::can_transfer::*;

// Const
const MPU_READ_INTERVAL: u64 = 10; // ms
const DEBUG_INTERVAL: u64 = 2000; // ms
const I2C_FREQ: Rate = Rate::from_khz(100);
const CAN_BAUD: BaudRate = BaudRate::B500K;

// Declare Wakeup signal 
static WAKE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static WAKE_SIGNAL_1: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static WAKE_SIGNAL_2: Signal<CriticalSectionRawMutex, [u8; 15]> = Signal::new();
static TEMP: Mutex<RefCell<f32>> = Mutex::new(RefCell::new(0.0));
static GYRO: Mutex<RefCell<Vector3d<f32>>> = Mutex::new(RefCell::new(Vector3d { x: 0.0, y: 0.0, z: 0.0 }));
static ACC: Mutex<RefCell<Vector3d<f32>>> = Mutex::new(RefCell::new(Vector3d { x: 0.0, y: 0.0, z: 0.0 }));
static WAKE_TIME: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(10));

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // Config peripherals
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let i2c = I2c::new(
        peripherals.I2C0,
        Config::default().with_frequency(I2C_FREQ),)
    .unwrap()
    .with_sda(peripherals.GPIO20)
    .with_scl(peripherals.GPIO21);
    let mpu = Mpu6050::new(i2c);
    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);
    let can_config = TwaiConfiguration::new(peripherals.TWAI0, peripherals.GPIO0, peripherals.GPIO1, CAN_BAUD, TwaiMode::Normal,);
    let can = CANTransfer::new(can_config);
    let led = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());

    // Init tasks
    spawner.spawn(run()).ok();                  // placeholder (change this if needed) currently used for debug purpose
    spawner.spawn(read_mpu(mpu)).ok();          // read MPU sensor
    spawner.spawn(send_can(can, led)).ok();     // send data through CAN bus

    // Init wakeup task signal
    let mut now = Instant::now();
    let mut task1_deadline = now + Duration::from_millis(MPU_READ_INTERVAL);
    let mut task2_deadline = now + Duration::from_millis(DEBUG_INTERVAL);
    loop {
        let time = critical_section::with(|cs| *WAKE_TIME.borrow(cs).borrow());
        if now >= task1_deadline {
            WAKE_SIGNAL_1.signal(());
            task1_deadline = now + Duration::from_millis(time);
        }
        else if now >= task2_deadline {
            WAKE_SIGNAL.signal(());
            task2_deadline = now + Duration::from_millis(DEBUG_INTERVAL);
        }
        now = Instant::now();
        sleep(1).await;
    }
}

#[embassy_executor::task]
async fn read_mpu(mut mpu: Mpu6050<I2c<'static, Blocking>>){
    let mut delay = Delay::new();
    mpu.init(&mut delay).unwrap();
    loop {
        WAKE_SIGNAL_1.wait().await;
        // get temp
        let temp = mpu.get_temp_raw().unwrap();
        let temp1 = mpu.get_temp().unwrap();
        // get gyro data, scaled with sensitivity 
        let gyro = mpu.get_gyro_raw().unwrap();
        let gyro1 = mpu.get_gyro().unwrap();
        let Vector3d { x: gyro_x, y: gyro_y, z: gyro_z } = gyro;
        // get accelerometer data, scaled with sensitivity
        let acc = mpu.get_acc_raw().unwrap();
        let acc1 = mpu.get_acc().unwrap();
        let Vector3d { x: acc_x, y: acc_y, z: acc_z } = acc;

        critical_section::with(|cs| {
            *TEMP.borrow(cs).borrow_mut() = temp1;
            *GYRO.borrow(cs).borrow_mut() = gyro1;
            *ACC.borrow(cs).borrow_mut()  = acc1;
        });

        let mut packet = [0u8; 15];
        packet[0..2].copy_from_slice(&temp.to_le_bytes());
        // Copy gyro
        packet[2..4].copy_from_slice(&gyro_x.to_le_bytes());
        packet[4..6].copy_from_slice(&gyro_y.to_le_bytes());
        packet[6..8].copy_from_slice(&gyro_z.to_le_bytes());
        // Copy acc
        packet[8..10].copy_from_slice(&acc_x.to_le_bytes());
        packet[10..12].copy_from_slice(&acc_y.to_le_bytes());
        packet[12..14].copy_from_slice(&acc_z.to_le_bytes());
        const X8: crc::Crc<u8> = crc::Crc::<u8>::new(&crc::CRC_8_AUTOSAR);
        packet[14] = X8.checksum(&packet);
        WAKE_SIGNAL_2.signal(packet);

    }
}

#[embassy_executor::task]
async fn run() {
    loop {
        WAKE_SIGNAL.wait().await;
        // get TEMP
        let temp = critical_section::with(|cs| *TEMP.borrow(cs).borrow());
        // get GYRO
        let gyro = critical_section::with(|cs| *GYRO.borrow(cs).borrow());
        // get ACC
        let acc = critical_section::with(|cs| *ACC.borrow(cs).borrow());
        // print to debug:
        defmt::println!("Temperature: {} Accelerometer: x={}, y={}, z={} Gyroscope: x={}, y={}, z={}", temp, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z);
    }
}

#[embassy_executor::task]
async fn send_can(mut can: Twai<'static, Blocking>, mut led: Output<'static>) {
    let mut previous_busoff: bool = false;
    loop {
        let data = WAKE_SIGNAL_2.wait().await;
        match can.is_bus_off(){
            true => {
                defmt::error!("Bus Off error, try to reconnect device!");
                led.toggle();
                critical_section::with(|cs| {*WAKE_TIME.borrow(cs).borrow_mut() = 1000;});
                previous_busoff = true;
            },
            false => {
                if previous_busoff {
                    previous_busoff = false;
                    defmt::println!("Bus Off Recovery");
                    led.set_low();
                    critical_section::with(|cs| {*WAKE_TIME.borrow(cs).borrow_mut() = 10;});
                }},
        }
        match CANTransfer::send_sensor_data(&mut can, &data) {
            Ok(()) => defmt::println!("All sensor data sent successfully"),
            Err(e) => defmt::error!("Failed to send sensor data: {:?}", e),
        }
    }
}
/// Sleep
pub async fn sleep(millis: u32) {
    Timer:: after(Duration::from_millis(millis as u64)).await;
}