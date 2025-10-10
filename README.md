# pmw3389-rs

An [embedded-hal 1.0](https://docs.rs/embedded-hal/1.0.0/embedded_hal/) compatible Rust driver for the **PixArt PMW3389 optical sensor**, commonly used in gaming mice and trackballs.

This crate provides a safe and ergonomic interface to initialize the sensor, configure CPI (counts per inch), read motion data, and manage power modes.

> ⚠️ **Note:** The PMW3389 requires an external SROM firmware binary (`pmw3389_srom.bin`).  
> Due to licensing restrictions, this file is **not included in this repository**.  
> Please obtain it from PixArt, an official distributor, or your hardware vendor.

---

## ✨ Features

- `embedded-hal` 1.0 SPI driver
- Sensor initialization (with SROM upload)
- Read motion data (`dx`, `dy`, `squal`)
- Configure CPI (50–16,000, step 50)
- Power-down mode
- Basic error handling (`Spi`, `Timeout`, `InvalidParam`, `InvalidResponse`)

---

## 📦 Installation

Add this crate to your `Cargo.toml`:

```toml
[dependencies]
pmw3389 = { git = "https://github.com/YOURNAME/pmw3389-rs" }
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
* [`cpi.rs`](examples/cpi.rs): Demonstrates setting CPI
* [`power_down.rs`](examples/power_down.rs): Demonstrates power-down mode

To build for RP2040 and convert to a UF2 image:

```bash
# Build the example for the RP2040 target
cargo build --release --example rp2040-zero --target thumbv6m-none-eabi

# Convert the produced ELF into a UF2 image (writes rp2040-zero.uf2 next to the ELF)
cargo run --bin rp2040-uf2 -- target/thumbv6m-none-eabi/release/examples/rp2040-zero
```

The conversion tool accepts an optional second argument if you want to override the
output path:

```bash
cargo run --bin rp2040-uf2 -- target/thumbv6m-none-eabi/release/examples/rp2040-zero \
    firmware/rp2040-zero.uf2
```

---

## ⚠️ SROM Firmware

This driver requires the **PMW3389 SROM binary**.

* It is **not distributed** due to licensing restrictions.
* You must obtain it from PixArt, your distributor, or your hardware vendor.
* Place it under `firmware/pmw3389_srom.bin` before building.

---

## 📜 License

This project is licensed under the [MIT License](LICENSE).

---

## 🙏 Acknowledgments

* [PixArt Imaging](https://www.pixart.com/) for the PMW3389 sensor
* The [embedded-hal](https://github.com/rust-embedded/embedded-hal) project
* Community projects that inspired this driver

```
