#![no_std]

pub mod registers;

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{Operation, SpiDevice};
use registers::Register;

/// Error type for PMW3389 driver
#[derive(Debug)]
pub enum Error<SpiErr> {
    /// Error from underlying SPI bus/device
    Spi(SpiErr),

    /// Invalid parameter passed to API
    InvalidParam,

    /// Unexpected or invalid response from sensor
    InvalidResponse,

    /// Timeout while waiting for device
    Timeout,
}

/// Motion data reported by the sensor
#[derive(Debug, Clone, Copy)]
pub struct Motion {
    pub delta_x: i16,
    pub delta_y: i16,
    pub squal: u8,
}

/// Main driver struct
pub struct Pmw3389<SPI, D> {
    spi: SPI,
    delay: D,
}

impl<SPI, D> Pmw3389<SPI, D>
where
    SPI: SpiDevice<u8>,
    D: DelayNs,
{
    /// Create a new driver instance
    pub fn new(spi: SPI, delay: D) -> Self {
        Self { spi, delay }
    }

    /// Initialize the sensor (reset, SROM download, etc.)
    pub fn init(&mut self, srom: &[u8]) -> Result<(), Error<SPI::Error>> {
        // 実装詳細は省略 (Power_Up_Reset, SROM download)
        Ok(())
    }

    /// Read product ID register (0x42 expected for PMW3389)
    pub fn product_id(&mut self) -> Result<u8, Error<SPI::Error>> {
        self.read_register(Register::ProductId)
    }

    /// Read revision ID register
    pub fn revision_id(&mut self) -> Result<u8, Error<SPI::Error>> {
        self.read_register(Register::RevisionId)
    }

    /// Read firmware (SROM) ID register
    pub fn firmware_id(&mut self) -> Result<u8, Error<SPI::Error>> {
        self.read_register(Register::SromId)
    }

    /// Set CPI (resolution)
    /// Valid range: 50..=16_000, step = 50
    pub fn set_cpi(&mut self, cpi: u16) -> Result<(), Error<SPI::Error>> {
        if cpi < 50 || cpi > 16_000 || cpi % 50 != 0 {
            return Err(Error::InvalidParam);
        }

        let cpi_reg = cpi / 50;
        let bytes = cpi_reg.to_le_bytes();

        self.write_register(Register::Cpi_L, bytes[0])?;
        self.write_register(Register::Cpi_H, bytes[1])?;
        Ok(())
    }

    /// Read motion data
    pub fn read_motion(&mut self) -> Result<Motion, Error<SPI::Error>> {
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
    pub fn power_down(&mut self) -> Result<(), Error<SPI::Error>> {
        self.write_register(Register::Shutdown, 0xB6)
    }

    // ---- Internal helpers ----

    fn read_register(&mut self, reg: Register) -> Result<u8, Error<SPI::Error>> {
        let addr = reg.addr() | 0x80;
        let mut buf = [0u8];
        self.spi
            .transaction(&mut [Operation::Write(&[addr]), Operation::Read(&mut buf)])
            .map_err(Error::Spi)?;
        Ok(buf[0])
    }

    fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<SPI::Error>> {
        let addr = reg.addr() & 0x7F;
        self.spi
            .transaction(&mut [Operation::Write(&[addr, value])])
            .map_err(Error::Spi)?;
        Ok(())
    }
}
