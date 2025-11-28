# pmw3389-rs

An [embedded-hal 1.0](https://docs.rs/embedded-hal/1.0.0/embedded_hal/) compatible Rust driver for the **PixArt PMW3389 optical sensor**, commonly used in gaming mice and trackballs.

This crate provides a safe and ergonomic interface to initialize the sensor, configure CPI (counts per inch), read motion data, and manage power modes.

---

## ✨ Features

- `embedded-hal` 1.0 SPI driver
- Sensor initialization (with SROM upload)
- Read motion data (`dx`, `dy`, `squal`)
- Configure CPI (50–16,000, step 50)
- Power-down mode
- Basic error handling (`Spi`, `Timeout`, `InvalidParam`, `InvalidResponse`)

---

## ⚠️ SROM Firmware Requirement

> **IMPORTANT:** This driver requires the `pmw3389_srom.bin` firmware to function.

Due to licensing restrictions, the firmware binary is **not included** in this repository. You must obtain it from an official source and make it available to your project at compile time.

*   **Source:** You must obtain the binary from PixArt, an official distributor, or your hardware vendor.
*   **Usage:** The examples in this repository expect the file to be at `firmware/pmw3389_srom.bin`, but the driver's `init` function simply takes the firmware as a slice `&[u8]`.

---

## 📦 Installation

Add this crate to your `Cargo.toml`:

```toml
[dependencies]
pmw3389 = { git = "https://github.com/MarchRaBBiT/pmw3389-rs" }
````

---

## 🚀 Usage

### Initialization

```rust
use pmw3389::Pmw3389;

// Load SROM firmware at compile time
const PMW3389_SROM: &[u8] = include_bytes!("../firmware/pmw3389_srom.bin");

// Create SPI + delay implementation first...
let mut sensor = Pmw3389::new(spi_dev, delay);

// Initialize the sensor with firmware
sensor.init(PMW3389_SROM)?;

// Read product ID (should be 0x42)
let pid = sensor.product_id()?;
```

### Reading motion data

```rust
let motion = sensor.read_motion()?;
defmt::info!("dx={}, dy={}, squal={}", motion.delta_x, motion.delta_y, motion.squal);
```

### Setting CPI

```rust
sensor.set_cpi(1600)?;
```

### Power down

```rust
sensor.power_down()?;
```

---

## 🧪 Examples

See the [`examples/`](examples) directory for runnable code:

* [`rp2040-zero.rs`](examples/rp2040-zero.rs): Basic init + motion readout
* [`waveshare-esp32s3.rs`](examples/waveshare-esp32s3.rs): Same flow for the Waveshare ESP32-S3 evaluation board (logs over USB-Serial-JTAG)
* [`waveshare-esp32c3.rs`](examples/waveshare-esp32c3.rs): Same flow for the Waveshare ESP32-C3 evaluation board (logs over USB-Serial-JTAG)
* [`cpi.rs`](examples/cpi.rs): Demonstrates setting CPI
* [`power_down.rs`](examples/power_down.rs): Demonstrates power-down mode

To build and run on RP2040 (via probe-rs):

```bash
cargo run --release --example rp2040-zero
```

To build for the Waveshare ESP32-C3 board (using the ESP toolchain and espflash):

```bash
cargo run --release --example waveshare-esp32c3 --target riscv32imc-unknown-none-elf
```

To build for the Waveshare ESP32-S3 board (using the ESP toolchain and espflash):

```bash
cargo run --release --example waveshare-esp32s3 --target xtensa-esp32s3-none-elf
```

Note: The default runner (configured in `Cargo.toml`) uses `probe-rs` for direct flashing. If you prefer generating a UF2 instead, install `elf2uf2-rs`, uncomment the `runner = "elf2uf2-rs"` line in `Cargo.toml`, and run the same command. A file like `target/thumbv6m-none-eabi/release/examples/rp2040-zero.uf2` will be produced for drag-and-drop deployment.

---

## 📜 License

This project is licensed under the [MIT License](LICENSE).

---

## 🙏 Acknowledgments

* [PixArt Imaging](https://www.pixart.com/) for the PMW3389 sensor
* The [embedded-hal](https://github.com/rust-embedded/embedded-hal) project
* Community projects that inspired this driver

```
