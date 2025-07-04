#![no_std]
#![no_main]

// peripherals-related imports
use esp_alloc as _;
use esp_hal::{
    clock::CpuClock, gpio::{Level, Output, OutputConfig}, rng::Rng, timer::timg::TimerGroup, tsens::Temperature
};

use esp_wifi::{
    init,
    wifi::{ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState},
    EspWifiController,
};

// embassy related imports
use embassy_executor::Spawner;
use embassy_net::{
    tcp::TcpSocket,
    Runner,
    {dns::DnsQueryType, Config as EmbassyNetConfig, StackResources},
};
use embassy_time::{Duration, Timer, Instant};

// Temperature sensor related imports
// use crate::bmp180_async::Bmp180;
mod bmp180_async;

// MQTT related imports
use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::reason_codes::ReasonCode,
    utils::rng_generator::CountingRng,
};

// Formatting related imports
use core::fmt::Write;
use heapless::String;

use heapless::spsc::Queue;

use esp_backtrace as _;
use log::{debug, error, info};

// use mpu6050_lib::lib::*;

// use micromath::vector::{Vector2d, Vector3d};

// mod mpu6050_lib{
// 	pub mod lib;
// 	pub mod mpu;
// 	pub mod bits;
// }

use embassy_sync::{blocking_mutex::raw::NoopRawMutex
                    , channel::Channel
                    , blocking_mutex::raw::CriticalSectionRawMutex
                    , signal::Signal};        // là kênh giao tiếp (channel) kiểu MPSC (multi-producer single-consumer)
use embedded_can::{Frame, Id}; 
use esp_hal::twai::{filter::SingleStandardFilter, *};

mod can {
    pub mod receive;
}
use can::receive::{ImuData, can_receiver, CANSetUp, QUEUE};
use critical_section::Mutex;
use core::cell::RefCell;

esp_bootloader_esp_idf::esp_app_desc!();


// static mut CHANNEL: Channel<NoopRawMutex, String<128>, 10> = TwaiOutbox::new();
// static QUEUE: Mutex<RefCell<Queue<String<128>, 32>>> = Mutex::new(RefCell::new(Queue::new()));
// Declare Wakeup signal 
static WAKE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

// maintains wifi connection, when it disconnects it tries to reconnect
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    debug!("Device capabilities: {:?}", controller.capabilities());
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!");
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                error!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

// A background task, to process network events - when new packets, they need to be processed, embassy-net, wraps smoltcp
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut rng = Rng::new(peripherals.RNG);

    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );

    let (controller, interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();
    let wifi_interface = interfaces.sta;

    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

    let config = EmbassyNetConfig::dhcpv4(Default::default());

    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(runner)).ok();

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    // wait until wifi connected
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address); // dhcp IP address
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    //setup CAN
    let can = CANSetUp::new(TwaiConfiguration::new(peripherals.TWAI0, peripherals.GPIO0, peripherals.GPIO1, BaudRate::B500K, TwaiMode::Normal,).into_async());
    let (can_rx, can_tx) = can.split();

    // let led = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());

    // let _ = spawner.spawn(blink_led(led));
    let _ = spawner.spawn(can_receiver(can_rx));
    // let _ = spawner.spawn(get_imu_data());

    loop {
        Timer::after(Duration::from_millis(1_000)).await;

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        let address = match stack
            .dns_query("app.coreiot.io", DnsQueryType::A)
            .await
            .map(|a| a[0])
        {
            Ok(address) => address,
            Err(e) => {
                error!("DNS lookup error: {e:?}");
                continue;
            }
        };

        let remote_endpoint = (address, 1883);
        info!("connecting...");
        let connection = socket.connect(remote_endpoint).await;
        if let Err(e) = connection {
            error!("connect error: {:?}", e);
            continue;
        }
        info!("connected!");

        let mut config = ClientConfig::new(
            rust_mqtt::client::client_config::MqttVersion::MQTTv5,
            CountingRng(20000),
        );
        config.add_max_subscribe_qos(rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1);
        config.add_username("vUTga2R9Qdwg0GHredeo"); // Set access token as username
        config.add_client_id("vUTga2R9Qdwg0GHredeo"); // Set access token as client ID
        config.max_packet_size = 128; // Increased to handle larger payload
        let mut recv_buffer = [0; 256]; // Increased buffer size
        let mut write_buffer = [0; 256]; // Increased buffer size

        let mut client =
            MqttClient::<_, 5, _>::new(socket, &mut write_buffer, 256, &mut recv_buffer, 256, config);

        match client.connect_to_broker().await {
            Ok(()) => {}
            Err(mqtt_error) => match mqtt_error {
                ReasonCode::NetworkError => {
                    error!("MQTT Network Error");
                    continue;
                }
                _ => {
                    error!("Other MQTT Error: {:?}", mqtt_error);
                    continue;
                }
            },
        }

        let mut now = Instant::now();
        let mut wakeup_deadline = now + Duration::from_secs(25);

        loop {

            let mut empty_queue = false;

            let imu_data = critical_section::with(|cs| {
                match QUEUE.borrow_ref_mut(cs).dequeue() {
                    Some(s) => s,
                    None => { empty_queue = true;
                            String::new() }
                }
            });

            if empty_queue { //sleep
                info!("empty queue!");
                now = Instant::now();
                if now >= wakeup_deadline { //25s elapsed
                    WAKE_SIGNAL.signal(());
                    info!("Wake Up!");
                    wakeup_deadline = now + Duration::from_secs(25); //restart 25s for wake up
                }
                sleep(100).await;
                continue;
            }

            now = Instant::now();
            wakeup_deadline = now + Duration::from_secs(25); //restart 25s for wake up

            match client
                .send_message(
                    "v1/devices/me/telemetry", // ThingsBoard telemetry topic
                    imu_data.as_bytes(),
                    rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1,
                    true,
                )
                .await
            {
                Ok(()) => info!("Telemetry sent successfully"),
                Err(mqtt_error) => match mqtt_error {
                    ReasonCode::NetworkError => {
                        error!("MQTT Network Error");
                        continue;
                    }
                    _ => {
                        error!("Other MQTT Error: {:?}", mqtt_error);
                        continue;
                    }
                },
            }
            // Timer::after(Duration::from_millis(3000)).await;
        }
    }
}

#[embassy_executor::task]
async fn blink_led(mut led_gpio: Output<'static>){
    led_gpio.set_high();
    loop {
        led_gpio.toggle();
        // info!("Task run");
        sleep(1000).await;
    }
}

pub async fn sleep(millis: u32) {
    Timer::after(Duration::from_millis(millis as u64)).await;
}
