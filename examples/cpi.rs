#![no_std]
#![no_main]

use defmt::{error, info, unwrap, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Config, Phase, Polarity, Spi};
use embassy_time::{Delay, Timer};
use panic_probe as _;
use pmw3389::Pmw3389;

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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Embassy initialized");

    let mut spi_config = Config::default();
    spi_config.frequency = 2_000_000;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.polarity = Polarity::IdleHigh;

    let spi = Spi::new(
        p.SPI0, p.PIN_2, // SCLK
        p.PIN_3, // MOSI
        p.PIN_4, // MISO
        p.DMA_CH0, p.DMA_CH1, spi_config,
    );

    let cs_pin = Output::new(p.PIN_5, Level::High);
    let mut sensor = unwrap!(Pmw3389::new(spi, cs_pin, Delay));

    info!("Initializing sensor...");
    if let Some(srom) = pmw3389_srom() {
        if let Err(e) = sensor.init(srom) {
            error!("Sensor init failed: {:?}", e);
            loop {
                Timer::after_millis(1000).await;
            }
        }
    } else {
        warn!("pmw3389_srom.bin not found; skipping firmware upload");
    }

    let cpis = [800u16, 1600u16, 3200u16];
    loop {
        for &cpi in &cpis {
            match sensor.set_cpi(cpi) {
                Ok(_) => info!("CPI set to {}", cpi),
                Err(e) => {
                    error!("Failed to set CPI: {:?}", e);
                    break;
                }
            }

            Timer::after_millis(2000).await;
        }
    }
}
