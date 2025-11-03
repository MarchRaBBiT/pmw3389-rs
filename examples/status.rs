#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use embedded_hal;
use embedded_hal::delay::DelayNs;
use hal::{pac, spi::Spi, uart::UartPeripheral};
use pmw3389::{Pmw3389, OperationMode};
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

    let mut itoa_buf = itoa::Buffer::new();

    loop {
        if let Ok(status) = sensor.read_status() {
            uart.write_full_blocking(b"Status: \r\n");

            if status.is_motion {
                uart.write_full_blocking(b"  Motion\r\n");
            } else {
                uart.write_full_blocking(b"  No Motion\r\n");
            }

            match status.operation_mode {
                OperationMode::Run => uart.write_full_blocking(b"  Run Mode\r\n"),
                OperationMode::Rest1 => uart.write_full_blocking(b"  Rest1 Mode\r\n"),
                OperationMode::Rest2 => uart.write_full_blocking(b"  Rest2 Mode\r\n"),
                OperationMode::Rest3 => uart.write_full_blocking(b"  Rest3 Mode\r\n"),
            }

            if status.is_self_test_running {
                uart.write_full_blocking(b"  Self Test Running\r\n");
            } else {
                uart.write_full_blocking(b"  Self Test Not Running\r\n");
            }

        } else {
            uart.write_full_blocking(b"Read status failed\r\n");
        }

        if let Ok(obs_data) = sensor.read_observation_data() {
            let mut buf = [0u8; 32];
            let mut cursor = 0;

            uart.write_full_blocking(b"Observation Data: \r\n");

            cursor += b"  RawDataSum: ".len();
            let s = itoa_buf.format(obs_data.raw_data_sum);
            buf[cursor..cursor + s.len()].copy_from_slice(s.as_bytes());
            cursor += s.len();
            buf[cursor..cursor + 2].copy_from_slice(b"\r\n");
            cursor += 2;
            uart.write_full_blocking(&buf[..cursor]);
            cursor = 0;

            cursor += b"  MaxRawData: ".len();
            let s = itoa_buf.format(obs_data.maximum_raw_data);
            buf[cursor..cursor + s.len()].copy_from_slice(s.as_bytes());
            cursor += s.len();
            buf[cursor..cursor + 2].copy_from_slice(b"\r\n");
            cursor += 2;
            uart.write_full_blocking(&buf[..cursor]);
            cursor = 0;

            cursor += b"  MinRawData: ".len();
            let s = itoa_buf.format(obs_data.minimum_raw_data);
            buf[cursor..cursor + s.len()].copy_from_slice(s.as_bytes());
            cursor += s.len();
            buf[cursor..cursor + 2].copy_from_slice(b"\r\n");
            cursor += 2;
            uart.write_full_blocking(&buf[..cursor]);
            cursor = 0;

            cursor += b"  ShutterLower: ".len();
            let s = itoa_buf.format(obs_data.shutter_lower);
            buf[cursor..cursor + s.len()].copy_from_slice(s.as_bytes());
            cursor += s.len();
            buf[cursor..cursor + 2].copy_from_slice(b"\r\n");
            cursor += 2;
            uart.write_full_blocking(&buf[..cursor]);
            cursor = 0;

            cursor += b"  ShutterUpper: ".len();
            let s = itoa_buf.format(obs_data.shutter_upper);
            buf[cursor..cursor + s.len()].copy_from_slice(s.as_bytes());
            cursor += s.len();
            buf[cursor..cursor + 2].copy_from_slice(b"\r\n");
            cursor += 2;
            uart.write_full_blocking(&buf[..cursor]);
            cursor = 0;

        } else {
            uart.write_full_blocking(b"Read observation data failed\r\n");
        }

        if let Ok(bist_result) = sensor.read_bist_result() {
            let mut buf = [0u8; 32];
            let mut cursor = 0;

            uart.write_full_blocking(b"BIST Result: ");
            let s = itoa_buf.format(bist_result);
            buf[cursor..cursor + s.len()].copy_from_slice(s.as_bytes());
            cursor += s.len();
            buf[cursor..cursor + 2].copy_from_slice(b"\r\n");
            cursor += 2;
            uart.write_full_blocking(&buf[..cursor]);
            cursor = 0;
        } else {
            uart.write_full_blocking(b"Read BIST result failed\r\n");
        }

        delay.delay_ms(1000);
    }
}

#[no_mangle]
pub extern "C" fn __pre_init() {
    // This function is called before `main`.
    // It's usually used for very early hardware initialization.
    // For now, we'll leave it empty to satisfy the linker.
}
