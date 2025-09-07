#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use hal::{pac, spi::Spi, uart::UartPeripheral};
use pmw3389::Pmw3389;
use rp2040_hal as hal;

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

    // UART
    let uart_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());
    let mut uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_115200_8_N_1,
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    // SPI
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

    uart.write_full_blocking(b"Entering power-down mode...\r\n");
    let _ = sensor.power_down();

    loop {
        // この後はモーションデータは取得できない
        cortex_m::asm::wfi(); // CPU も省電力待機
    }
}
