# Bare-Metal "no-std" Dual-ESP32-C3 Telemetry Pipeline

This project implements a bare-metal, `no_std` Rust-based telemetry pipeline using two **ESP32-C3** boards: a **Sensor Node** and a **Gateway Node**. The system streams motion data from an MPU-6050 Inertial Measurement Unit (IMU) to an MQTT broker, adhering to the requirements outlined in the "Bare-Metal `no-std` Dual-ESP32 Telemetry Pipeline" specification.

## Project Overview

The system consists of two ESP32-C3-based nodes:
- **Sensor Node**: Interfaces with an MPU-6050 IMU via I2C (400 kHz) to collect motion data (temperature, accelerometer, and gyroscope) and transmits it over TWAI (CAN bus) at 500 kbit/s to the Gateway Node.
- **Gateway Node**: Receives CAN data, connects to a Wi-Fi network as a station (STA), and forwards the data to an MQTT broker (e.g., `app.coreiot.io`) via TCP. The Gateway Node enters light-sleep mode when no CAN traffic is detected and wakes every 25 seconds if no CAN traffic is present.

Both nodes operate in a bare-metal environment using the Embassy executor for interrupt-driven task management, with no RTOS. The project uses nightly Rust (≥1.78) with custom panic and allocation error handlers, and logging is performed via `defmt` or UART.

## System Architecture

### Sensor Node
- **Hardware**: ESP32-C3
- **Interfaces**:
  - I2C (400 kHz) to MPU-6050 for reading temperature, accelerometer, and gyroscope data.
  - TWAI (CAN bus, 500 kbit/s) for transmitting data to the Gateway Node.
- **Functionality**:
  - Reads raw IMU data every 10 ms, scales it, and formats it into a 15-byte packet (2 bytes temperature, 6 bytes gyroscope, 6 bytes accelerometer, 1 byte CRC).
  - Splits the packet into three CAN frames (ID 0x120) with identifiers for temperature (0x00), accelerometer (0x01), and gyroscope (0x02).
  - Uses a CRC-8-AUTOSAR checksum for data integrity.
  - Toggles an LED (GPIO8) to indicate bus-off errors and adjusts the read interval (10 ms normal, 1000 ms during bus-off).
- **Files**:
  - `src/main.rs`: Main application logic, initializes peripherals, and spawns tasks for IMU reading, CAN transmission, and debugging.
  - `src/can_transfer.rs`: Handles CAN frame transmission, splitting the 15-byte packet into three frames.
  - `src/mpu6050_lib/lib.rs`, `src/mpu6050_lib/mpu.rs`, `src/mpu6050_lib/bits.rs`: MPU-6050 driver for I2C communication and data processing.
  - `Cargo.toml`: Project configuration for the Sensor Node.

### Gateway Node
- **Hardware**: ESP32-C3
- **Interfaces**:
  - TWAI (CAN bus, 500 kbit/s) for receiving data from the Sensor Node.
  - Wi-Fi STA for connecting to an MQTT broker via TCP.
- **Functionality**:
  - Receives CAN frames, reassembles them into a 15-byte packet, and verifies the CRC.
  - Formats the data into JSON and publishes it to the MQTT topic `v1/devices/me/telemetry` on `app.coreiot.io:1883`. (https://app.coreiot.io/login)
  - Enters light-sleep mode when no CAN traffic is detected, waking every 25 seconds if no traffic is present.
  - Uses Embassy for task management and `esp_wifi` for Wi-Fi connectivity.
- **Files**:
  - `src/main.rs`: Main application logic, initializes Wi-Fi, CAN, and MQTT client, and manages sleep/wake cycles.
  - `src/can/receive.rs`: Handles CAN frame reception, reassembly, and JSON formatting.
  - `Cargo.toml`: Project configuration for the Gateway Node.

### Common Protocol
- **Data Format**: 15-byte packet (2 bytes temperature, 6 bytes gyroscope, 6 bytes accelerometer, 1 byte CRC-8-AUTOSAR).
- **CAN Frames**: Three 8-byte frames with standard ID 0x120, each prefixed with a chunk identifier (0x00, 0x01, 0x02).
- **MQTT Payload**: JSON format, e.g., `{"temperature":23.5,"ax":0.12,"ay":0.45,"az":-0.98,"gx":1.23,"gy":0.56,"gz":-0.34}`.

## Wiring

### Sensor Node
- **MPU-6050**:
  - SDA: GPIO20
  - SCL: GPIO21
  - VCC: 3.3V
  - GND: GND
- **CAN Transceiver**:
  - TX: GPIO0
  - RX: GPIO1
- **LED**: GPIO8 (for bus-off indication)
- **Power**: 3.3V supply

### Gateway Node
- **CAN Transceiver**:
  - TX: GPIO0
  - RX: GPIO1
- **Wi-Fi**: Configured via SSID and PASSWORD environment variables.
- **Power**: 3.3V supply

## Flashing Instructions

Refer to this book for a detailed instruction: https://docs.espressif.com/projects/rust/book/installation/riscv.html

1. **Install Toolchain**:
   - Install `espup` and ensure the Rust environment is set up for ESP32-C3.
   - Verify with `cargo espflash --version`.

2. **Clone Repository**:
   ```bash
   git clone <repository-url>
   cd <repository>
   ```

3. **Build and Flash**:
   - Sensor Node:
     ```bash
     cd sensor-node
     cargo run --release
     ```
   - Gateway Node:
     ```bash
     cd gateway-node
     cargo run --release
     ```
   - Note: `cargo run --release` builds and flashes the firmware using `espflash`. Ensure `espflash` is configured in your `Cargo.toml` or environment.

4. **Monitor Output**:
   ```bash
   cargo espflash monitor
   ```

## MQTT Broker Setup

1. Configure the Gateway Node with your MQTT broker credentials:
   - Set `SSID` and `PASSWORD` environment variables in `gateway-node/.env` for Wi-Fi.
   - Update the MQTT client configuration in `gateway-node/src/main.rs` (username and client ID, e.g., `vUTga2R9Qdwg0GHredeo`).
2. Ensure the broker (e.g., `app.coreiot.io:1883`) is accessible and supports MQTT v5.

## Testing and Verification

- **Loopback Test**: Run TWAI self-test on each board to verify CAN functionality.
- **IMU Driver Test**: Use host-side unit tests with a mock I2C interface to validate the MPU-6050 driver.
- **Latency Test**: Measure CAN ISR and MQTT publish latency using a scope on GPIO toggles.
- **Sleep Cycle Test**: Simulate CAN pulses to verify the Gateway Node's sleep/wake behavior (wakes every 25 seconds if no CAN traffic).
- **Long-Run Test**: Run for 8 hours, logging metrics via UART or a metrics socket.

## Project Structure

```
.
├── sensor-node/
│   ├── src/
│   │   ├── main.rs
│   │   ├── can_transfer.rs
│   │   ├── mpu6050_lib/
│   │   │   ├── lib.rs
│   │   │   ├── mpu.rs
│   │   │   ├── bits.rs
│   ├── Cargo.toml
├── gateway-node/
│   ├── src/
│   │   ├── main.rs
│   │   ├── can/
│   │   │   ├── receive.rs
│   ├── Cargo.toml
├── README.md
```

## Notes

- The project uses `esp_alloc` for heap allocation on the Gateway Node and no heap on the Sensor Node.
- Logging is implemented via `defmt` for the Sensor Node and `log` for the Gateway Node.
- The Sensor Node uses a fixed 10 ms read interval, adjustable to 1000 ms during CAN bus-off.
- The Gateway Node enters light-sleep when no CAN traffic is detected and wakes every 25 seconds if no traffic is present, with a 25-second MQTT reconnection cycle.
- Ensure `Cargo.toml` for each node includes dependencies like `esp-hal`, `embassy-executor`, and `defmt` (Sensor Node) or `log` (Gateway Node), and configures `espflash` for `cargo run`.
- A mechanism is still required to handle CAN bus-off state recovery.
