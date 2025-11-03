#![no_std]
#![no_main]

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::USB;
use embassy_rp::spi::{Config, Phase, Polarity, Spi};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Delay, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use futures::join;
use itoa;
use panic_probe as _;
use pmw3389::Pmw3389;

// SROM for PMW3389
const PMW3389_SROM: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/pmw3389_srom.bin"));

bind_interrupts! { struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
} }

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Embassy initialized");

    // Create the USB driver
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.manufacturer = Some("Embassy");
    config.product = Some("PMW3389 Example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    config.device_class = 0x02;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    static mut CONFIG_DESC: [u8; 256] = [0; 256];
    static mut BOS_DESC: [u8; 256] = [0; 256];
    static mut MSOS_DESC: [u8; 256] = [0; 256];
    static mut CONTROL_BUF: [u8; 128] = [0; 128];
    static mut STATE: State = State::new();

    let mut device_builder = Builder::new(
        driver,
        config,
        unsafe { &mut CONFIG_DESC },
        unsafe { &mut BOS_DESC },
        unsafe { &mut MSOS_DESC },
        unsafe { &mut CONTROL_BUF },
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut device_builder, unsafe { &mut STATE }, 64);

    // Build the builder.
    let mut usb = device_builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Set up SPI
    let mut spi_config = Config::default();
    spi_config.frequency = 2_000_000;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.polarity = Polarity::IdleHigh;
    let spi = Spi::new(
        p.SPI0,
        p.PIN_2, // SCLK
        p.PIN_3, // MOSI
        p.PIN_4, // MISO
        p.DMA_CH0,
        p.DMA_CH1,
        spi_config,
    );

    let cs_pin = Output::new(p.PIN_5, Level::High);

    // Create sensor driver
    let mut sensor = unwrap!(Pmw3389::new(spi, cs_pin, Delay));

    // --- Main Loop ---
    let task = async {
        loop {
            class.wait_connection().await;
            info!("USB connected");

            if let Err(e) = sensor.init(PMW3389_SROM) {
                info!("Sensor init failed: {:?}", e);
                loop {}
            }

            let pid = unwrap!(sensor.product_id());
            info!("Product ID: {=u8:#x}", pid);

            let mut dx_buf = itoa::Buffer::new();
            let mut dy_buf = itoa::Buffer::new();

            loop {
                if let Ok(motion) = sensor.read_motion() {
                    let dx_str = dx_buf.format(motion.delta_x);
                    let dy_str = dy_buf.format(motion.delta_y);

                    info!("Motion: dx={}, dy={}", dx_str, dy_str);
                }

                Timer::after_millis(100).await;
            }
        }
    };

    join!(usb_fut, task);
}