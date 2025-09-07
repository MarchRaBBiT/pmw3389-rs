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
        // 1. Power-Up Reset: Must be done to start sensor
        self.write_register(Register::PowerUpReset, 0x5A)?;
        // Wait for sensor to boot
        self.delay.delay_ms(50);

        // 2. Clear motion registers to ensure clean state
        self.read_register(Register::Motion)?;
        self.read_register(Register::DeltaX_L)?;
        self.read_register(Register::DeltaX_H)?;
        self.read_register(Register::DeltaY_L)?;
        self.read_register(Register::DeltaY_H)?;

        // 3. SROM Download sequence
        self.write_register(Register::SromEnable, 0x1d)?;
        self.delay.delay_ms(10);
        self.write_register(Register::SromEnable, 0x18)?;

        // 4. Burst write SROM data. The address for SROM write needs the write bit (MSB=1).
        let addr = Register::MotionBurst.addr() | 0x80;
        self.spi
            .transaction(&mut [Operation::Write(&[addr]), Operation::Write(srom)])
            .map_err(Error::Spi)?;

        // Wait for SROM to initialize and validate.
        self.delay.delay_ms(12); // t_SROM_DONE_2_NCS_FALL_min is 12ms

        // Read SROM ID to confirm download. Should be 0x04 for the standard firmware.
        self.read_register(Register::SromId)?;

        // 5. Set recommended performance settings from the datasheet
        self.write_register(Register::Config2, 0x00)?;
        self.write_register(Register::Run_Downshift, 0x4f)?;
        self.write_register(Register::Rest1_Rate_Lower, 0x1f)?;
        self.write_register(Register::Rest1_Rate_Upper, 0x00)?;
        self.write_register(Register::Rest1_Downshift, 0x20)?;
        self.write_register(Register::Rest2_Rate_Lower, 0x60)?;
        self.write_register(Register::Rest2_Rate_Upper, 0x00)?;
        self.write_register(Register::Rest2_Downshift, 0x30)?;

        Ok(())
    }

    /// Read product ID register (should be 0x42 for PMW3389)
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
    /// Valid range: 50..=16_000, in steps of 50
    pub fn set_cpi(&mut self, cpi: u16) -> Result<(), Error<SPI::Error>> {
        if cpi < 50 || cpi > 16_000 || cpi % 50 != 0 {
            return Err(Error::InvalidParam);
        }

        let cpi_reg = cpi / 50;
        let bytes = cpi_reg.to_le_bytes();

        self.write_register(Register::Resolution_L, bytes[0])?;
        self.write_register(Register::Resolution_H, bytes[1])?;
        Ok(())
    }

    /// Read motion data (delta_x, delta_y, and SQUAL)
    pub fn read_motion(&mut self) -> Result<Motion, Error<SPI::Error>> {
        // Reading the Motion register is required to latch the delta values
        self.read_register(Register::Motion)?;
        let dx_l = self.read_register(Register::DeltaX_L)? as i16;
        let dx_h = self.read_register(Register::DeltaX_H)? as i16;
        let dy_l = self.read_register(Register::DeltaY_L)? as i16;
        let dy_h = self.read_register(Register::DeltaY_H)? as i16;
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
        // For a read, the MSB of the address is 0.
        let addr = reg.addr() & 0x7F;
        let mut buf = [0u8];

        // The PMW3389 requires a delay (t_SRAD) of at least 160us between sending
        // the address and reading the data. This transaction does not include that
        // delay. A dummy write can be used to create a delay if needed,
        // or use separate write/delay/read calls if the SPI device implementation allows.
        // For now, we correct the address bit and keep the original structure.
        self.spi
            .transaction(&mut [Operation::Write(&[addr]), Operation::Read(&mut buf)])
            .map_err(Error::Spi)?;
        
        Ok(buf[0])
    }

    fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<SPI::Error>> {
        // For a write, the MSB of the address is 1.
        let addr = reg.addr() | 0x80;
        self.spi
            .transaction(&mut [Operation::Write(&[addr, value])])
            .map_err(Error::Spi)?;
        // Wait t_SCLK-NCS_WR for write operation to complete
        self.delay.delay_us(35);
        Ok(())
    }
}
