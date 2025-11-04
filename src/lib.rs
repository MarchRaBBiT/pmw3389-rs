#![no_std]

pub mod registers;

use core::result::Result;
use core::result::Result::{Err, Ok};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;
use registers::Register;

/// Error type for PMW3389 driver
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<SpiErr, PinErr> {
    /// Error from underlying SPI bus
    Spi(SpiErr),
    /// Error from chip select pin
    Pin(PinErr),

    /// Invalid parameter passed to API
    InvalidParam,

    /// Unexpected or invalid response from sensor
    InvalidResponse,

    /// Timeout while waiting for device
    Timeout,
}

/// Motion data reported by the sensor
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Motion {
    pub delta_x: i16,
    pub delta_y: i16,
    pub squal: u8,
}

/// Main driver struct
pub struct Pmw3389<SPI, CS, D> {
    spi: SPI,
    cs: CS,
    delay: D,
}

impl<SPI, CS, D> Pmw3389<SPI, CS, D>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    D: DelayNs,
{
    /// Create a new driver instance
    pub fn new(spi: SPI, mut cs: CS, delay: D) -> Result<Self, Error<SPI::Error, CS::Error>> {
        cs.set_high().map_err(Error::Pin)?;
        Ok(Self { spi, cs, delay })
    }

    /// Initialize the sensor (reset, SROM download, etc.)
    pub fn init(&mut self, srom: &[u8]) -> Result<(), Error<SPI::Error, CS::Error>> {
        // 1. Power-Up Reset
        self.write_register(Register::PowerUpReset, 0x5A)?;
        self.delay.delay_ms(50);

        // 2. Clear motion registers
        self.read_register(Register::Motion)?;
        self.read_register(Register::DeltaXL)?;
        self.read_register(Register::DeltaXH)?;
        self.read_register(Register::DeltaYL)?;
        self.read_register(Register::DeltaYH)?;

        // 3. SROM Download sequence
        self.write_register(Register::SromEnable, 0x1d)?;
        self.delay.delay_ms(10);
        self.write_register(Register::SromEnable, 0x18)?;
        self.delay.delay_us(180); // Wait for SROM download to be ready

        // 4. Burst write SROM data
        self.cs.set_low().map_err(Error::Pin)?;
        let addr = Register::MotionBurst.addr() | 0x80;
        self.spi.write(&[addr]).map_err(Error::Spi)?;
        self.delay.delay_us(15);

        for byte in srom {
            self.spi.write(&[*byte]).map_err(Error::Spi)?;
            self.delay.delay_us(15);
        }
        self.cs.set_high().map_err(Error::Pin)?;
        self.delay.delay_us(200); // Wait for SROM to initialize

        // Read SROM ID to confirm download.
        self.read_register(Register::SromId)?;

        // 5. Set recommended performance settings
        self.write_register(Register::Config2, 0x00)?;
        self.write_register(Register::RunDownshift, 0x4f)?;
        self.write_register(Register::Rest1RateLower, 0x1f)?;
        self.write_register(Register::Rest1RateUpper, 0x00)?;
        self.write_register(Register::Rest1Downshift, 0x20)?;
        self.write_register(Register::Rest2RateLower, 0x60)?;
        self.write_register(Register::Rest2RateUpper, 0x00)?;
        self.write_register(Register::Rest2Downshift, 0x30)?;

        Ok(())
    }

    /// Read product ID register (should be 0x42 for PMW3389)
    pub fn product_id(&mut self) -> Result<u8, Error<SPI::Error, CS::Error>> {
        self.read_register(Register::ProductId)
    }

    /// Read revision ID register
    pub fn revision_id(&mut self) -> Result<u8, Error<SPI::Error, CS::Error>> {
        self.read_register(Register::RevisionId)
    }

    /// Read firmware (SROM) ID register
    pub fn firmware_id(&mut self) -> Result<u8, Error<SPI::Error, CS::Error>> {
        self.read_register(Register::SromId)
    }

    /// Set CPI (resolution)
    /// Valid range: 50..=16_000, in steps of 50
    pub fn set_cpi(&mut self, cpi: u16) -> Result<(), Error<SPI::Error, CS::Error>> {
        if cpi < 50 || cpi > 16_000 || cpi % 50 != 0 {
            return Err(Error::InvalidParam);
        }

        let cpi_reg = cpi / 50;
        let bytes = cpi_reg.to_le_bytes();

        self.write_register(Register::ResolutionL, bytes[0])?;
        self.write_register(Register::ResolutionH, bytes[1])?;
        Ok(())
    }

    /// Read motion data (delta_x, delta_y, and SQUAL)
    pub fn read_motion(&mut self) -> Result<Motion, Error<SPI::Error, CS::Error>> {
        // Reading the Motion register is required to latch the delta values
        self.read_register(Register::Motion)?;
        let dx_l = self.read_register(Register::DeltaXL)? as i16;
        let dx_h = self.read_register(Register::DeltaXH)? as i16;
        let dy_l = self.read_register(Register::DeltaYL)? as i16;
        let dy_h = self.read_register(Register::DeltaYH)? as i16;
        let squal = self.read_register(Register::Squal)?;

        Ok(Motion {
            delta_x: (dx_h << 8) | dx_l,
            delta_y: (dy_h << 8) | dy_l,
            squal,
        })
    }

    /// Put the sensor into shutdown (power-down) mode
    pub fn power_down(&mut self) -> Result<(), Error<SPI::Error, CS::Error>> {
        self.write_register(Register::Shutdown, 0xB6)
    }

    #[cfg(feature = "device-status")]
    pub fn read_status(&mut self) -> Result<DeviceStatus, Error<SPI::Error, CS::Error>> {
        let motion_reg = self.read_register(Register::Motion)?;
        Ok(DeviceStatus::from_reg(motion_reg))
    }

    // ---- Internal helpers ----

    fn read_register(&mut self, reg: Register) -> Result<u8, Error<SPI::Error, CS::Error>> {
        self.cs.set_low().map_err(Error::Pin)?;
        let addr = reg.addr() & 0x7F;
        self.spi.write(&[addr]).map_err(Error::Spi)?;
        self.delay.delay_us(160); // t_SRAD
        let mut buf = [0u8];
        self.spi.read(&mut buf).map_err(Error::Spi)?;
        self.cs.set_high().map_err(Error::Pin)?;
        self.delay.delay_us(20);
        Ok(buf[0])
    }

    fn write_register(
        &mut self,
        reg: Register,
        value: u8,
    ) -> Result<(), Error<SPI::Error, CS::Error>> {
        self.cs.set_low().map_err(Error::Pin)?;
        let addr = reg.addr() | 0x80;
        self.spi.write(&[addr, value]).map_err(Error::Spi)?;
        self.cs.set_high().map_err(Error::Pin)?;
        self.delay.delay_us(35);
        Ok(())
    }
}

#[cfg(feature = "device-status")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OperationMode {
    Run,
    Rest1,
    Rest2,
    Rest3,
}

#[cfg(feature = "device-status")]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceStatus {
    pub is_motion: bool,
    pub operation_mode: OperationMode,
    pub is_self_test_running: bool,
}

#[cfg(feature = "device-status")]
impl DeviceStatus {
    pub fn from_reg(reg_val: u8) -> Self {
        let is_motion = (reg_val & 0x80) != 0;
        let op_mode_bits = (reg_val >> 4) & 0b111;
        let operation_mode = match op_mode_bits {
            0b000 => OperationMode::Run,
            0b001 => OperationMode::Rest1,
            0b010 => OperationMode::Rest2,
            _ => OperationMode::Rest3, // Includes 0b011 and other values
        };
        let is_self_test_running = (reg_val & 0x04) != 0;

        Self {
            is_motion,
            operation_mode,
            is_self_test_running,
        }
    }
}

#[cfg(feature = "device-status")]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ObservationData {
    pub raw_data_sum: u8,
    pub maximum_raw_data: u8,
    pub minimum_raw_data: u8,
    pub shutter_lower: u8,
    pub shutter_upper: u8,
}

#[cfg(feature = "device-status")]
impl<SPI, CS, D> Pmw3389<SPI, CS, D>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    D: DelayNs,
{
    pub fn read_observation_data(&mut self) -> Result<ObservationData, Error<SPI::Error, CS::Error>> {
        let raw_data_sum = self.read_register(Register::RawDataSum)?;
        let maximum_raw_data = self.read_register(Register::MaximumRawData)?;
        let minimum_raw_data = self.read_register(Register::MinimumRawData)?;
        let shutter_lower = self.read_register(Register::ShutterLower)?;
        let shutter_upper = self.read_register(Register::ShutterUpper)?;

        Ok(ObservationData {
            raw_data_sum,
            maximum_raw_data,
            minimum_raw_data,
            shutter_lower,
            shutter_upper,
        })
    }

    /// Read the Built-In Self-Test (BIST) result.
    /// A value of 0x00 indicates success.
    #[cfg(feature = "device-status")]
    pub fn read_bist_result(&mut self) -> Result<u8, Error<SPI::Error, CS::Error>> {
        self.read_register(Register::BistResult)
    }
}
