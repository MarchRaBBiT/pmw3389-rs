#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use hal::{pac, spi::Spi, uart::UartPeripheral};
use pmw3389::Pmw3389;
use rp2040_hal as hal;

// build.rs 経由で埋め込み
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

    // UART 出力 (GPIO0, GPIO1)
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
    let spi_dev = hal::spi::SpiDevice::new(spi, cs_pin);

    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut sensor = Pmw3389::new(spi_dev, delay);

    uart.write_full_blocking(b"Init...\r\n");
    let _ = sensor.init(PMW3389_SROM);

    // CPI を 800, 1600, 3200 に切り替えながら動作確認
    for &cpi in &[800u16, 1600u16, 3200u16] {
        if let Err(e) = sensor.set_cpi(cpi) {
            uart.write_full_blocking(b"Set CPI failed\r\n");
            core::hint::spin_loop();
        } else {
            uart.write_full_blocking(b"CPI set to ");
            // 簡易数値出力（itoa を使うのが望ましい）
            let _ = uart.write_full_blocking(&[(cpi / 100) as u8 + b'0']);
            uart.write_full_blocking(b"00\r\n");
        }

        sensor.delay.delay_ms(2000);
    }

    loop {}
}
