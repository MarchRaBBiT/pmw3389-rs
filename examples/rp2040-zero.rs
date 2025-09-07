#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use embedded_hal::spi::SpiDevice;
use hal::{spi::Spi, uart::UartPeripheral};
use pmw3389::Pmw3389;
use rp2040_hal as hal;

// build.rs または firmware ディレクトリから埋め込み
const PMW3389_SROM: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/pmw3389_srom.bin"));

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        12_000_000,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());
    let mut uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_115200_8_N_1,
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    // SPI: GPIO2=SCLK, GPIO3=MOSI, GPIO4=MISO, GPIO5=CSn
    let spi = Spi::new(pac.SPI0, (pins.gpio2, pins.gpio3, pins.gpio4)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        2_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );

    let cs_pin = pins.gpio5.into_push_pull_output();
    let mut spi_dev = hal::spi::SpiDevice::new(spi, cs_pin);

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut sensor = Pmw3389::new(spi_dev, delay);

    uart.write_full_blocking(b"Init PMW3389...\r\n");

    if let Err(e) = sensor.init(PMW3389_SROM) {
        uart.write_full_blocking(b"Init failed\r\n");
        loop {}
    }

    let pid = sensor.product_id().unwrap();
    let _ = uart.write_full_blocking(b"Product ID=");
    let _ = uart.write_full_blocking(&[pid]);
    let _ = uart.write_full_blocking(b"\r\n");

    loop {
        if let Ok(motion) = sensor.read_motion() {
            // 簡易出力（本当は itoa 等で整数→文字列化）
            uart.write_full_blocking(b"Motion dx=");
            uart.write_full_blocking(&[motion.delta_x as u8]);
            uart.write_full_blocking(b", dy=");
            uart.write_full_blocking(&[motion.delta_y as u8]);
            uart.write_full_blocking(b"\r\n");
        }

        delay.delay_ms(100);
    }
}
