#![cfg_attr(not(test), no_std)]

#[cfg(test)]
extern crate std;

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
    pub fn read_observation_data(
        &mut self,
    ) -> Result<ObservationData, Error<SPI::Error, CS::Error>> {
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

#[cfg(test)]
mod test_util {
    use super::*;
    use crate::registers::Register;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::digital::OutputPin;
    use embedded_hal::spi::SpiBus;
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::rc::Rc;

    #[derive(Debug, PartialEq, Eq)]
    pub enum SpiTransaction {
        Write(Vec<u8>),
        Read(Vec<u8>),
    }

    pub struct MockSpi {
        expectations: Rc<RefCell<VecDeque<SpiTransaction>>>,
    }

    impl MockSpi {
        pub fn new(expectations: Rc<RefCell<VecDeque<SpiTransaction>>>) -> Self {
            Self { expectations }
        }
    }

    impl SpiBus<u8> for MockSpi {
        type Error = ();

        fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            let txn = self
                .expectations
                .borrow_mut()
                .pop_front()
                .expect("unexpected SPI read");
            match txn {
                SpiTransaction::Read(data) => {
                    assert_eq!(
                        data.len(),
                        words.len(),
                        "expected read length {} but got {}",
                        data.len(),
                        words.len()
                    );
                    words.copy_from_slice(&data);
                    Ok(())
                }
                SpiTransaction::Write(data) => {
                    panic!("expected SPI write {:?} but read() was called", data);
                }
            }
        }

        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            let txn = self
                .expectations
                .borrow_mut()
                .pop_front()
                .expect("unexpected SPI write");
            match txn {
                SpiTransaction::Write(expected) => {
                    assert_eq!(expected, words, "SPI write mismatch");
                    Ok(())
                }
                SpiTransaction::Read(data) => {
                    panic!(
                        "expected SPI read {:?} but write({:?}) was called",
                        data, words
                    );
                }
            }
        }

        fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
            panic!("unexpected SPI transfer call");
        }

        fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
            panic!("unexpected SPI transfer_in_place call");
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub enum PinAction {
        High,
        Low,
    }

    #[derive(Clone)]
    pub struct MockPin {
        log: Rc<RefCell<Vec<PinAction>>>,
    }

    impl MockPin {
        pub fn new(log: Rc<RefCell<Vec<PinAction>>>) -> Self {
            Self { log }
        }
    }

    impl OutputPin for MockPin {
        type Error = ();

        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.log.borrow_mut().push(PinAction::Low);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.log.borrow_mut().push(PinAction::High);
            Ok(())
        }
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub enum DelayCall {
        Ns(u32),
        Us(u32),
        Ms(u32),
    }

    pub struct MockDelay {
        calls: Rc<RefCell<Vec<DelayCall>>>,
    }

    impl MockDelay {
        pub fn new(calls: Rc<RefCell<Vec<DelayCall>>>) -> Self {
            Self { calls }
        }
    }

    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.calls.borrow_mut().push(DelayCall::Ns(ns));
        }

        fn delay_us(&mut self, us: u32) {
            self.calls.borrow_mut().push(DelayCall::Us(us));
        }

        fn delay_ms(&mut self, ms: u32) {
            self.calls.borrow_mut().push(DelayCall::Ms(ms));
        }
    }

    pub fn new_spi(
        expectations: Vec<SpiTransaction>,
    ) -> (MockSpi, Rc<RefCell<VecDeque<SpiTransaction>>>) {
        let deque = Rc::new(RefCell::new(VecDeque::from(expectations)));
        (MockSpi::new(deque.clone()), deque)
    }

    pub fn new_pin_log() -> (MockPin, Rc<RefCell<Vec<PinAction>>>) {
        let log = Rc::new(RefCell::new(Vec::new()));
        (MockPin::new(log.clone()), log)
    }

    pub fn new_delay_log() -> (MockDelay, Rc<RefCell<Vec<DelayCall>>>) {
        let calls = Rc::new(RefCell::new(Vec::new()));
        (MockDelay::new(calls.clone()), calls)
    }

    pub fn push_write(expectations: &mut Vec<SpiTransaction>, reg: Register, value: u8) {
        expectations.push(SpiTransaction::Write(vec![reg.addr() | 0x80, value]));
    }

    pub fn push_read(expectations: &mut Vec<SpiTransaction>, reg: Register, value: u8) {
        expectations.push(SpiTransaction::Write(vec![reg.addr() & 0x7F]));
        expectations.push(SpiTransaction::Read(vec![value]));
    }

    pub fn push_write_raw(expectations: &mut Vec<SpiTransaction>, bytes: Vec<u8>) {
        expectations.push(SpiTransaction::Write(bytes));
    }
}

#[cfg(test)]
mod tests {
    use super::test_util::*;
    use super::*;
    use crate::registers::Register;

    #[test]
    fn new_sets_cs_high() {
        let (spi, spi_state) = new_spi(Vec::new());
        let (pin, pin_log) = new_pin_log();
        let (delay, delay_log) = new_delay_log();

        let driver = Pmw3389::new(spi, pin, delay);
        assert!(driver.is_ok());
        assert!(spi_state.borrow().is_empty());
        assert!(delay_log.borrow().is_empty());

        let log = pin_log.borrow();
        assert_eq!(log.len(), 1);
        assert_eq!(log[0], PinAction::High);
    }

    #[test]
    fn set_cpi_writes_resolution_registers() {
        let mut expectations = Vec::new();
        push_write(&mut expectations, Register::ResolutionL, 0x10);
        push_write(&mut expectations, Register::ResolutionH, 0x00);

        let (spi, spi_state) = new_spi(expectations);
        let (pin, pin_log) = new_pin_log();
        let (delay, delay_log) = new_delay_log();

        let mut driver = Pmw3389::new(spi, pin, delay).unwrap();
        pin_log.borrow_mut().clear(); // ignore initial high

        driver.set_cpi(800).unwrap();
        assert!(spi_state.borrow().is_empty());

        let log = pin_log.borrow();
        assert_eq!(
            log.as_slice(),
            &[
                PinAction::Low,
                PinAction::High,
                PinAction::Low,
                PinAction::High
            ]
        );
        let delay_entries = delay_log.borrow();
        assert_eq!(
            delay_entries
                .iter()
                .filter(|call| matches!(call, DelayCall::Us(35)))
                .count(),
            2
        );
    }

    #[test]
    fn set_cpi_rejects_invalid_values() {
        let (spi, _) = new_spi(Vec::new());
        let (pin, _) = new_pin_log();
        let (delay, _) = new_delay_log();
        let mut driver = Pmw3389::new(spi, pin, delay).unwrap();

        for &value in &[40u16, 16_050, 55] {
            let err = driver.set_cpi(value).unwrap_err();
            assert!(matches!(err, Error::InvalidParam));
        }
    }

    #[test]
    fn read_motion_combines_bytes() {
        let mut expectations = Vec::new();
        push_read(&mut expectations, Register::Motion, 0x00);
        push_read(&mut expectations, Register::DeltaXL, 0xFE);
        push_read(&mut expectations, Register::DeltaXH, 0xFF);
        push_read(&mut expectations, Register::DeltaYL, 0x05);
        push_read(&mut expectations, Register::DeltaYH, 0x00);
        push_read(&mut expectations, Register::Squal, 0x33);

        let (spi, spi_state) = new_spi(expectations);
        let (pin, pin_log) = new_pin_log();
        let (delay, delay_log) = new_delay_log();

        let mut driver = Pmw3389::new(spi, pin, delay).unwrap();
        pin_log.borrow_mut().clear();

        let motion = driver.read_motion().unwrap();
        assert_eq!(motion.delta_x, -2);
        assert_eq!(motion.delta_y, 5);
        assert_eq!(motion.squal, 0x33);

        assert!(spi_state.borrow().is_empty());

        let log = pin_log.borrow();
        assert_eq!(log.len(), 12);
        assert!(log.iter().enumerate().all(|(idx, action)| {
            if idx % 2 == 0 {
                *action == PinAction::Low
            } else {
                *action == PinAction::High
            }
        }));

        let delay_entries = delay_log.borrow();
        assert_eq!(
            delay_entries
                .iter()
                .filter(|call| matches!(call, DelayCall::Us(160)))
                .count(),
            6
        );
        assert_eq!(
            delay_entries
                .iter()
                .filter(|call| matches!(call, DelayCall::Us(20)))
                .count(),
            6
        );
    }

    #[test]
    fn power_down_writes_shutdown_register() {
        let mut expectations = Vec::new();
        push_write(&mut expectations, Register::Shutdown, 0xB6);
        let (spi, spi_state) = new_spi(expectations);
        let (pin, pin_log) = new_pin_log();
        let (delay, delay_log) = new_delay_log();

        let mut driver = Pmw3389::new(spi, pin, delay).unwrap();
        pin_log.borrow_mut().clear();

        driver.power_down().unwrap();
        assert!(spi_state.borrow().is_empty());
        assert_eq!(
            pin_log.borrow().as_slice(),
            &[PinAction::Low, PinAction::High]
        );
        assert_eq!(
            delay_log
                .borrow()
                .iter()
                .filter(|call| matches!(call, DelayCall::Us(35)))
                .count(),
            1
        );
    }

    #[test]
    fn product_id_reads_correct_register() {
        let mut expectations = Vec::new();
        push_read(&mut expectations, Register::ProductId, 0x42);

        let (spi, spi_state) = new_spi(expectations);
        let (pin, pin_log) = new_pin_log();
        let (delay, _) = new_delay_log();

        let mut driver = Pmw3389::new(spi, pin, delay).unwrap();
        pin_log.borrow_mut().clear();

        let pid = driver.product_id().unwrap();
        assert_eq!(pid, 0x42);
        assert!(spi_state.borrow().is_empty());
    }

    #[test]
    fn init_performs_expected_sequence() {
        let srom = [0xAA, 0x55, 0x11];

        let mut expectations = Vec::new();
        push_write(&mut expectations, Register::PowerUpReset, 0x5A);
        for reg in &[
            Register::Motion,
            Register::DeltaXL,
            Register::DeltaXH,
            Register::DeltaYL,
            Register::DeltaYH,
        ] {
            push_read(&mut expectations, *reg, 0x00);
        }
        push_write(&mut expectations, Register::SromEnable, 0x1D);
        push_write(&mut expectations, Register::SromEnable, 0x18);
        push_write_raw(&mut expectations, vec![Register::MotionBurst.addr() | 0x80]);
        for byte in srom {
            push_write_raw(&mut expectations, vec![byte]);
        }
        push_read(&mut expectations, Register::SromId, 0xA5);
        for &(reg, value) in &[
            (Register::Config2, 0x00),
            (Register::RunDownshift, 0x4F),
            (Register::Rest1RateLower, 0x1F),
            (Register::Rest1RateUpper, 0x00),
            (Register::Rest1Downshift, 0x20),
            (Register::Rest2RateLower, 0x60),
            (Register::Rest2RateUpper, 0x00),
            (Register::Rest2Downshift, 0x30),
        ] {
            push_write(&mut expectations, reg, value);
        }

        let (spi, spi_state) = new_spi(expectations);
        let (pin, pin_log) = new_pin_log();
        let (delay, delay_log) = new_delay_log();

        let mut driver = Pmw3389::new(spi, pin, delay).unwrap();
        pin_log.borrow_mut().clear();

        driver.init(&srom).unwrap();
        assert!(spi_state.borrow().is_empty());

        let log = pin_log.borrow();
        assert_eq!(log.len(), 36);
        assert!(log.iter().enumerate().all(|(idx, action)| {
            if idx % 2 == 0 {
                *action == PinAction::Low
            } else {
                *action == PinAction::High
            }
        }));

        let delays = delay_log.borrow();
        assert!(delays.contains(&DelayCall::Ms(50)));
        assert!(delays.contains(&DelayCall::Ms(10)));
        assert!(delays.contains(&DelayCall::Us(180)));
        assert!(delays.contains(&DelayCall::Us(200)));
        let write_delays = delays
            .iter()
            .filter(|call| matches!(call, DelayCall::Us(35)))
            .count();
        assert_eq!(write_delays, 10);
    }
}

#[cfg(test)]
mod register_tests {
    use super::registers::Register;

    #[test]
    fn register_addresses_match_datasheet() {
        assert_eq!(Register::ProductId.addr(), 0x00);
        assert_eq!(Register::Motion.addr(), 0x02);
        assert_eq!(Register::ResolutionH.addr(), 0x0F);
        assert_eq!(Register::SromEnable.addr(), 0x13);
        assert_eq!(Register::Shutdown.addr(), 0x3B);
        assert_eq!(Register::MotionBurst.addr(), 0x50);
    }
}

#[cfg(all(test, feature = "device-status"))]
mod status_tests {
    use super::test_util::*;
    use super::*;
    use crate::registers::Register;

    #[test]
    fn device_status_from_reg_decodes_flags() {
        let status = DeviceStatus::from_reg(0b1001_0100);
        assert!(status.is_motion);
        assert!(matches!(status.operation_mode, OperationMode::Rest2));
        assert!(status.is_self_test_running);

        let status = DeviceStatus::from_reg(0b0000_0000);
        assert!(!status.is_motion);
        assert!(matches!(status.operation_mode, OperationMode::Run));
        assert!(!status.is_self_test_running);
    }

    #[test]
    fn read_observation_data_reads_all_fields() {
        let mut expectations = Vec::new();
        push_read(&mut expectations, Register::RawDataSum, 10);
        push_read(&mut expectations, Register::MaximumRawData, 20);
        push_read(&mut expectations, Register::MinimumRawData, 5);
        push_read(&mut expectations, Register::ShutterLower, 30);
        push_read(&mut expectations, Register::ShutterUpper, 40);

        let (spi, spi_state) = new_spi(expectations);
        let (pin, _) = new_pin_log();
        let (delay, delay_log) = new_delay_log();

        let mut driver = Pmw3389::new(spi, pin, delay).unwrap();

        let obs = driver.read_observation_data().unwrap();
        assert_eq!(obs.raw_data_sum, 10);
        assert_eq!(obs.maximum_raw_data, 20);
        assert_eq!(obs.minimum_raw_data, 5);
        assert_eq!(obs.shutter_lower, 30);
        assert_eq!(obs.shutter_upper, 40);
        assert!(spi_state.borrow().is_empty());

        assert_eq!(
            delay_log
                .borrow()
                .iter()
                .filter(|call| matches!(call, DelayCall::Us(160)))
                .count(),
            5
        );
    }

    #[test]
    fn read_bist_result_reads_correct_register() {
        let mut expectations = Vec::new();
        push_read(&mut expectations, Register::BistResult, 0x7F);
        let (spi, spi_state) = new_spi(expectations);
        let (pin, _) = new_pin_log();
        let (delay, _) = new_delay_log();

        let mut driver = Pmw3389::new(spi, pin, delay).unwrap();
        let bist = driver.read_bist_result().unwrap();
        assert_eq!(bist, 0x7F);
        assert!(spi_state.borrow().is_empty());
    }
}
