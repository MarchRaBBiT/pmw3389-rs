#![no_std]
#![no_main]

use core::fmt::Debug;

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    spi::master::{Config as SpiConfig, Spi},
    spi::Mode,
    time::Rate,
    timer::timg::TimerGroup,
    Config,
};
use esp_hal_embassy as embassy;
use esp_println::println;
use itoa;
use pmw3389::Pmw3389;

// Provide SROM bytes only when the firmware image is available.
#[cfg(pmw3389_has_srom)]
fn pmw3389_srom() -> Option<&'static [u8]> {
    Some(include_bytes!(concat!(
        env!("OUT_DIR"),
        "/pmw3389_srom.bin"
    )))
}

#[cfg(not(pmw3389_has_srom))]
fn pmw3389_srom() -> Option<&'static [u8]> {
    None
}

#[embassy::main]
async fn main(_spawner: Spawner) {
    let config = Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    embassy::init(timg0.timer0);

    // SPI pinout for the Waveshare ESP32-S3 evaluation board.
    // SCLK: GPIO12, MOSI: GPIO11, MISO: GPIO13, CS: GPIO10
    let sclk = peripherals.GPIO12;
    let mosi = peripherals.GPIO11;
    let miso = peripherals.GPIO13;
    let cs_pin = Output::new(peripherals.GPIO10, Level::High, OutputConfig::default());

    let spi_config = SpiConfig::default()
        .with_frequency(Rate::from_mhz(2))
        .with_mode(Mode::_3);
    let spi = Spi::new(peripherals.SPI2, spi_config)
        .expect("failed to init SPI")
        .with_sck(sclk)
        .with_mosi(mosi)
        .with_miso(miso);

    // Create sensor driver
    let mut sensor = must(Pmw3389::new(spi, cs_pin, esp_hal::delay::Delay::new()));

    println!("PMW3389 sample on Waveshare ESP32-S3 (esp-hal-embassy)");

    if let Some(srom) = pmw3389_srom() {
        must(sensor.init(srom));
    } else {
        println!("pmw3389_srom.bin not found; skipping firmware upload");
    }

    let pid = must(sensor.product_id());
    println!("Product ID: 0x{:02x}", pid);

    let mut dx_buf = itoa::Buffer::new();
    let mut dy_buf = itoa::Buffer::new();

    loop {
        if let Ok(motion) = sensor.read_motion() {
            let dx_str = dx_buf.format(motion.delta_x);
            let dy_str = dy_buf.format(motion.delta_y);
            println!("Motion: dx={}, dy={}", dx_str, dy_str);
        }

        Timer::after_millis(100).await;
    }
}

// Minimal helper to reduce repetitive error handling without pulling in defmt on Xtensa.
fn must<T, E: Debug>(res: Result<T, E>) -> T {
    match res {
        Ok(v) => v,
        Err(e) => {
            println!("Fatal error: {:?}", e);
            loop {}
        }
    }
}
