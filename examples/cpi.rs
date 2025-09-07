#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use embedded_hal;
use embedded_hal::delay::DelayNs;
use hal::{pac, spi::Spi, uart::UartPeripheral};
use itoa;
use pmw3389::Pmw3389;
use rp2040_hal as hal;
use rp2040_hal::fugit::RateExtU32;
use rp2040_hal::prelude::*;
use rp2040_hal::uart::UartConfig;

// build.rs 経由で埋め込み
const PMW3389_SROM: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/pmw3389_srom.bin"));

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
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

    // UART 出力 (GPIO0, GPIO1)
    let uart_pins = (
        pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
        pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
    );
    let uart_config = UartConfig::new(
        115_200u32.Hz(),
        hal::uart::DataBits::Eight,
        None,
        hal::uart::StopBits::One,
    );
    let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(uart_config, clocks.peripheral_clock.freq())
        .unwrap();

    // SPI: GPIO2=SCLK, GPIO3=MOSI, GPIO4=MISO, GPIO5=CSn
    let spi_sclk = pins.gpio2.into_function::<hal::gpio::FunctionSpi>();
    let spi_mosi = pins.gpio3.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();

    let spi = Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        2_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );
    let cs_pin = pins.gpio5.into_push_pull_output();

    let mut delay = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut sensor = Pmw3389::new(spi, cs_pin, delay).unwrap();

    uart.write_full_blocking(b"Init...\r\n");
    if let Err(_e) = sensor.init(PMW3389_SROM) {
        uart.write_full_blocking(b"Init failed\r\n");
        loop {}
    }

    // CPI を 800, 1600, 3200 に切り替えながら動作確認
    let mut cpi_buf = itoa::Buffer::new();
    for &cpi in &[800u16, 1600u16, 3200u16] {
        if let Err(_e) = sensor.set_cpi(cpi) {
            uart.write_full_blocking(b"Set CPI failed\r\n");
            loop {}
        } else {
            let cpi_str = cpi_buf.format(cpi);
            uart.write_full_blocking(b"CPI set to ");
            uart.write_full_blocking(cpi_str.as_bytes());
            uart.write_full_blocking(b"\r\n");
        }

        delay.delay_ms(2_000);
    }

    loop {}
}
