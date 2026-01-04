#![no_std]

// MAX31865 driver inspired by Adafruit_MAX31865 (BSD license).

const MAX31865_CONFIG_REG: u8 = 0x00;
const MAX31865_RTDMSB_REG: u8 = 0x01;
#[allow(dead_code)]
const MAX31865_RTDLSB_REG: u8 = 0x02;
const MAX31865_HFAULTMSB_REG: u8 = 0x03;
const MAX31865_HFAULTLSB_REG: u8 = 0x04;
const MAX31865_LFAULTMSB_REG: u8 = 0x05;
const MAX31865_LFAULTLSB_REG: u8 = 0x06;
const MAX31865_FAULTSTAT_REG: u8 = 0x07;

const MAX31865_CONFIG_BIAS: u8 = 0x80;
const MAX31865_CONFIG_MODEAUTO: u8 = 0x40;
const MAX31865_CONFIG_1SHOT: u8 = 0x20;
const MAX31865_CONFIG_3WIRE: u8 = 0x10;
const MAX31865_CONFIG_FAULTSTAT: u8 = 0x02;
const MAX31865_CONFIG_FILT50HZ: u8 = 0x01;

// Callendar–Van Dusen coefficients used by Adafruit library.
const RTD_A: f32 = 3.9083e-3;
const RTD_B: f32 = -5.775e-7;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum WireCount {
    Two,
    Three,
    Four,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FilterHz {
    Hz50,
    Hz60,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FaultCycle {
    None,
    Auto,
    ManualRun,
    ManualFinish,
}

bitflags::bitflags! {
    /// Raw fault bits from the FAULTSTAT register.
    pub struct FaultStatus: u8 {
        const HIGHTHRESH = 0x80;
        const LOWTHRESH  = 0x40;
        const REFINLOW   = 0x20;
        const REFINHIGH  = 0x10;
        const RTDINLOW   = 0x08;
        const OVUV       = 0x04;
    }
}

/// Human-friendly messages (Adafruit-style)
pub fn fault_message(f: FaultStatus) -> &'static str {
    if f.contains(FaultStatus::HIGHTHRESH) { "RTD High Threshold" }
    else if f.contains(FaultStatus::LOWTHRESH) { "RTD Low Threshold" }
    else if f.contains(FaultStatus::REFINLOW) { "REFIN- > 0.85 x Bias" }
    else if f.contains(FaultStatus::REFINHIGH) { "REFIN- < 0.85 x Bias - FORCE- open" }
    else if f.contains(FaultStatus::RTDINLOW) { "RTDIN- < 0.85 x Bias - FORCE- open" }
    else if f.contains(FaultStatus::OVUV) { "Under/Over voltage" }
    else { "Unknown fault" }
}


/// Nominal RTD resistance (ohms at 0°C), e.g. 100Ω (PT100), 1000Ω (PT1000).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RtdNominal(pub f32);

impl RtdNominal {
    pub const PT100: Self = Self(100.0);
    pub const PT1000: Self = Self(1000.0);
}

/// Reference resistor value (ohms), typically 430Ω for PT100 boards, 4300Ω for PT1000 boards.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RefResistor(pub f32);

impl RefResistor {
    pub const OHM_430: Self = Self(430.0);
    pub const OHM_4300: Self = Self(4300.0);
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Config {
    pub wires: WireCount,
    pub filter: FilterHz,
    pub rtd_nominal: RtdNominal,
    pub ref_resistor: RefResistor,
}

impl Config {
    pub const fn pt100_3wire() -> Self {
        Self {
            wires: WireCount::Three,
            filter: FilterHz::Hz60,
            rtd_nominal: RtdNominal::PT100,
            ref_resistor: RefResistor::OHM_430,
        }
    }

    pub const fn pt1000_3wire() -> Self {
        Self {
            wires: WireCount::Three,
            filter: FilterHz::Hz60,
            rtd_nominal: RtdNominal::PT1000,
            ref_resistor: RefResistor::OHM_4300,
        }
    }
}

/// Driver error.
#[derive(Debug)]
pub enum Error<SpiE> {
    Spi(SpiE),
    InvalidConfig,
}

// No Display or std::error::Error impls in no_std

/// Blocking MAX31865 driver (embedded-hal).
pub struct Max31865<SPI> {
    spi: SPI,
    cfg: Config,
}

impl<SPI> Max31865<SPI>
where
    SPI: embedded_hal::spi::SpiDevice<u8>,
{
    pub fn new(spi: SPI, cfg: Config) -> Self {
        Self { spi, cfg }
    }

    pub fn free(self) -> (SPI, Config) {
        (self.spi, self.cfg)
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    pub fn set_config(&mut self, cfg: Config) {
        self.cfg = cfg;
    }

    /// Initialize chip to a sane default similar to Adafruit `begin()`:
    /// wires, bias off, autoconvert off, thresholds wide, clear faults.
    pub fn init(&mut self) -> Result<(), Error<SPI::Error>> {
        self.set_wires(self.cfg.wires)?;
        self.enable_bias(false)?;
        self.auto_convert(false)?;
        self.enable_50hz(self.cfg.filter == FilterHz::Hz50)?;
        self.set_thresholds(0, 0xFFFF)?;
        self.clear_fault()?;
        Ok(())
    }

    pub async fn read_fault<D: embedded_hal_async::delay::DelayNs>(
        &mut self, cycle: FaultCycle, delay: &mut D
    ) -> Result<FaultStatus, Error<SPI::Error>> {
        if cycle != FaultCycle::None {
            let mut cfg_reg = self.read_u8(MAX31865_CONFIG_REG)?;
            cfg_reg &= 0x11; // keep wire + filter bits, same as Adafruit

            match cycle {
                FaultCycle::Auto => {
                    self.write_u8(MAX31865_CONFIG_REG, cfg_reg | 0b1000_0100)?;
                    delay.delay_ms(1).await;
                }
                FaultCycle::ManualRun => {
                    self.write_u8(MAX31865_CONFIG_REG, cfg_reg | 0b1000_1000)?;
                    return Ok(FaultStatus::empty());
                }
                FaultCycle::ManualFinish => {
                    self.write_u8(MAX31865_CONFIG_REG, cfg_reg | 0b1000_1100)?;
                    return Ok(FaultStatus::empty());
                }
                FaultCycle::None => {}
            }
        }

        let raw = self.read_u8(MAX31865_FAULTSTAT_REG)?;
        Ok(FaultStatus::from_bits_truncate(raw))
    }

    pub fn clear_fault(&mut self) -> Result<(), Error<SPI::Error>> {
        let mut t = self.read_u8(MAX31865_CONFIG_REG)?;
        t &= !0x2C;
        t |= MAX31865_CONFIG_FAULTSTAT;
        self.write_u8(MAX31865_CONFIG_REG, t)?;
        Ok(())
    }

    pub fn enable_bias(&mut self, on: bool) -> Result<(), Error<SPI::Error>> {
        let mut t = self.read_u8(MAX31865_CONFIG_REG)?;
        if on {
            t |= MAX31865_CONFIG_BIAS;
        } else {
            t &= !MAX31865_CONFIG_BIAS;
        }
        self.write_u8(MAX31865_CONFIG_REG, t)?;
        Ok(())
    }

    pub fn auto_convert(&mut self, on: bool) -> Result<(), Error<SPI::Error>> {
        let mut t = self.read_u8(MAX31865_CONFIG_REG)?;
        if on {
            t |= MAX31865_CONFIG_MODEAUTO;
        } else {
            t &= !MAX31865_CONFIG_MODEAUTO;
        }
        self.write_u8(MAX31865_CONFIG_REG, t)?;
        Ok(())
    }

    pub fn enable_50hz(&mut self, on: bool) -> Result<(), Error<SPI::Error>> {
        let mut t = self.read_u8(MAX31865_CONFIG_REG)?;
        if on {
            t |= MAX31865_CONFIG_FILT50HZ;
        } else {
            t &= !MAX31865_CONFIG_FILT50HZ;
        }
        self.write_u8(MAX31865_CONFIG_REG, t)?;
        Ok(())
    }

    pub fn set_wires(&mut self, wires: WireCount) -> Result<(), Error<SPI::Error>> {
        let mut t = self.read_u8(MAX31865_CONFIG_REG)?;
        match wires {
            WireCount::Three => t |= MAX31865_CONFIG_3WIRE,
            WireCount::Two | WireCount::Four => t &= !MAX31865_CONFIG_3WIRE,
        }
        self.write_u8(MAX31865_CONFIG_REG, t)?;
        Ok(())
    }

    pub fn set_thresholds(&mut self, lower_raw: u16, upper_raw: u16) -> Result<(), Error<SPI::Error>> {
        self.write_u8(MAX31865_LFAULTLSB_REG, (lower_raw & 0xFF) as u8)?;
        self.write_u8(MAX31865_LFAULTMSB_REG, (lower_raw >> 8) as u8)?;
        self.write_u8(MAX31865_HFAULTLSB_REG, (upper_raw & 0xFF) as u8)?;
        self.write_u8(MAX31865_HFAULTMSB_REG, (upper_raw >> 8) as u8)?;
        Ok(())
    }

    pub fn lower_threshold(&mut self) -> Result<u16, Error<SPI::Error>> {
        self.read_u16(MAX31865_LFAULTMSB_REG)
    }

    pub fn upper_threshold(&mut self) -> Result<u16, Error<SPI::Error>> {
        self.read_u16(MAX31865_HFAULTMSB_REG)
    }

    /// One-shot RTD read (matches Adafruit timing).
    pub fn read_rtd<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<u16, Error<SPI::Error>> {
        self.clear_fault()?;
        self.enable_bias(true)?;
        delay.delay_ms(10);

        let mut t = self.read_u8(MAX31865_CONFIG_REG)?;
        t |= MAX31865_CONFIG_1SHOT;
        self.write_u8(MAX31865_CONFIG_REG, t)?;

        // Adafruit uses 65ms
        delay.delay_ms(65);

        let mut rtd = self.read_u16(MAX31865_RTDMSB_REG)?;

        self.enable_bias(false)?;

        // Remove fault bit
        rtd >>= 1;
        Ok(rtd)
    }

    pub fn read_temperature<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<f32, Error<SPI::Error>> {
        let raw = self.read_rtd(delay)?;
        Ok(calculate_temperature(
            raw,
            self.cfg.rtd_nominal.0,
            self.cfg.ref_resistor.0,
        ))
    }

    // ---- low-level register I/O ----

    fn read_u8(&mut self, addr: u8) -> Result<u8, Error<SPI::Error>> {
        let mut buf = [0u8; 1];
        self.read_n(addr, &mut buf)?;
        Ok(buf[0])
    }

    fn read_u16(&mut self, addr: u8) -> Result<u16, Error<SPI::Error>> {
        let mut buf = [0u8; 2];
        self.read_n(addr, &mut buf)?;
        Ok(((buf[0] as u16) << 8) | (buf[1] as u16))
    }

    fn read_n(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        use embedded_hal::spi::Operation;

        let a = addr & 0x7F; // ensure top bit is not set
        let w = [a];

        let mut ops = [Operation::Write(&w), Operation::Read(buf)];
        self.spi.transaction(&mut ops).map_err(Error::Spi)?;
        Ok(())
    }

    fn write_u8(&mut self, addr: u8, data: u8) -> Result<(), Error<SPI::Error>> {
        let a = addr | 0x80; // ensure top bit is set
        let w = [a, data];
        self.spi.write(&w).map_err(Error::Spi)?;
        Ok(())
    }
}

/// Pure function, usable by both blocking + async APIs.
pub fn calculate_temperature(rtd_raw: u16, rtd_nominal: f32, ref_resistor: f32) -> f32 {
    // Convert raw to resistance
    let mut rt = rtd_raw as f32;
    rt /= 32768.0;
    rt *= ref_resistor;

    // Quadratic approximation for >= 0°C
    let z1 = -RTD_A;
    let z2 = RTD_A * RTD_A - (4.0 * RTD_B);
    let z3 = (4.0 * RTD_B) / rtd_nominal;
    let z4 = 2.0 * RTD_B;

    let mut temp = z2 + (z3 * rt);
    temp = (libm::sqrtf(temp) + z1) / z4;

    if temp >= 0.0 {
        return temp;
    }

    // Polynomial for < 0°C (same as Adafruit)
    rt /= rtd_nominal;
    rt *= 100.0;

    let mut rpoly = rt;
    let mut t = -242.02;
    t += 2.2228 * rpoly;
    rpoly *= rt; // ^2
    t += 2.5859e-3 * rpoly;
    rpoly *= rt; // ^3
    t -= 4.8260e-6 * rpoly;
    rpoly *= rt; // ^4
    t -= 2.8183e-8 * rpoly;
    rpoly *= rt; // ^5
    t += 1.5243e-10 * rpoly;

    t
}

//
// Optional async driver
//

#[cfg(feature = "async")]
pub struct Max31865Async<SPI> {
    spi: SPI,
    cfg: Config,
}

#[cfg(feature = "async")]
impl<SPI> Max31865Async<SPI>
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
{

    pub fn new(spi: SPI, cfg: Config) -> Self {
        Self { spi, cfg }
    }

    pub fn free(self) -> (SPI, Config) {
        (self.spi, self.cfg)
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    pub fn set_config(&mut self, cfg: Config) {
        self.cfg = cfg;
    }

    pub async fn init(&mut self) -> Result<(), Error<SPI::Error>> {
        self.set_wires(self.cfg.wires).await?;
        self.enable_bias(false).await?;
        self.auto_convert(false).await?;
        self.enable_50hz(self.cfg.filter == FilterHz::Hz50).await?;
        self.set_thresholds(0, 0xFFFF).await?;
        self.clear_fault().await?;
        Ok(())
    }

    pub async fn read_rtd<D: embedded_hal_async::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<u16, Error<SPI::Error>> {
        self.clear_fault().await?;
        self.enable_bias(true).await?;
        delay.delay_ms(10).await;

        let mut t = self.read_u8(MAX31865_CONFIG_REG).await?;
        t |= MAX31865_CONFIG_1SHOT;
        self.write_u8(MAX31865_CONFIG_REG, t).await?;

        delay.delay_ms(65).await;

        let mut rtd = self.read_u16(MAX31865_RTDMSB_REG).await?;
        self.enable_bias(false).await?;

        rtd >>= 1;
        Ok(rtd)
    }

    pub async fn read_temperature<D: embedded_hal_async::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<f32, Error<SPI::Error>> {
        let raw = self.read_rtd(delay).await?;
        Ok(calculate_temperature(
            raw,
            self.cfg.rtd_nominal.0,
            self.cfg.ref_resistor.0,
        ))
    }

    pub async fn read_fault<D: embedded_hal_async::delay::DelayNs>(
        &mut self, cycle: FaultCycle, delay: &mut D
    ) -> Result<FaultStatus, Error<SPI::Error>> {
        if cycle != FaultCycle::None {
            let mut cfg_reg = self.read_u8(MAX31865_CONFIG_REG).await?;
            cfg_reg &= 0x11;

            match cycle {
                FaultCycle::Auto => {
                    self.write_u8(MAX31865_CONFIG_REG, cfg_reg | 0b1000_0100).await?;
                    delay.delay_ms(1).await;
                }
                FaultCycle::ManualRun => {
                    self.write_u8(MAX31865_CONFIG_REG, cfg_reg | 0b1000_1000).await?;
                    return Ok(FaultStatus::empty());
                }
                FaultCycle::ManualFinish => {
                    self.write_u8(MAX31865_CONFIG_REG, cfg_reg | 0b1000_1100).await?;
                    return Ok(FaultStatus::empty());
                }
                FaultCycle::None => {}
            }
        }

        let raw = self.read_u8(MAX31865_FAULTSTAT_REG).await?;
        Ok(FaultStatus::from_bits_truncate(raw))
    }

    pub async fn clear_fault(&mut self) -> Result<(), Error<SPI::Error>> {
        let mut t = self.read_u8(MAX31865_CONFIG_REG).await?;
        t &= !0x2C;
        t |= MAX31865_CONFIG_FAULTSTAT;
        self.write_u8(MAX31865_CONFIG_REG, t).await?;
        Ok(())
    }

    pub async fn enable_bias(&mut self, on: bool) -> Result<(), Error<SPI::Error>> {
        let mut t = self.read_u8(MAX31865_CONFIG_REG).await?;
        if on {
            t |= MAX31865_CONFIG_BIAS;
        } else {
            t &= !MAX31865_CONFIG_BIAS;
        }
        self.write_u8(MAX31865_CONFIG_REG, t).await?;
        Ok(())
    }

    pub async fn auto_convert(&mut self, on: bool) -> Result<(), Error<SPI::Error>> {
        let mut t = self.read_u8(MAX31865_CONFIG_REG).await?;
        if on {
            t |= MAX31865_CONFIG_MODEAUTO;
        } else {
            t &= !MAX31865_CONFIG_MODEAUTO;
        }
        self.write_u8(MAX31865_CONFIG_REG, t).await?;
        Ok(())
    }

    pub async fn enable_50hz(&mut self, on: bool) -> Result<(), Error<SPI::Error>> {
        let mut t = self.read_u8(MAX31865_CONFIG_REG).await?;
        if on {
            t |= MAX31865_CONFIG_FILT50HZ;
        } else {
            t &= !MAX31865_CONFIG_FILT50HZ;
        }
        self.write_u8(MAX31865_CONFIG_REG, t).await?;
        Ok(())
    }

    pub async fn set_wires(&mut self, wires: WireCount) -> Result<(), Error<SPI::Error>> {
        let mut t = self.read_u8(MAX31865_CONFIG_REG).await?;
        match wires {
            WireCount::Three => t |= MAX31865_CONFIG_3WIRE,
            WireCount::Two | WireCount::Four => t &= !MAX31865_CONFIG_3WIRE,
        }
        self.write_u8(MAX31865_CONFIG_REG, t).await?;
        Ok(())
    }

    pub async fn set_thresholds(&mut self, lower_raw: u16, upper_raw: u16) -> Result<(), Error<SPI::Error>> {
        self.write_u8(MAX31865_LFAULTLSB_REG, (lower_raw & 0xFF) as u8).await?;
        self.write_u8(MAX31865_LFAULTMSB_REG, (lower_raw >> 8) as u8).await?;
        self.write_u8(MAX31865_HFAULTLSB_REG, (upper_raw & 0xFF) as u8).await?;
        self.write_u8(MAX31865_HFAULTMSB_REG, (upper_raw >> 8) as u8).await?;
        Ok(())
    }

    // ---- low-level register I/O ----

    async fn read_u8(&mut self, addr: u8) -> Result<u8, Error<SPI::Error>> {
        let mut buf = [0u8; 1];
        self.read_n(addr, &mut buf).await?;
        Ok(buf[0])
    }

    async fn read_u16(&mut self, addr: u8) -> Result<u16, Error<SPI::Error>> {
        let mut buf = [0u8; 2];
        self.read_n(addr, &mut buf).await?;
        Ok(((buf[0] as u16) << 8) | (buf[1] as u16))
    }

    async fn read_n(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        use embedded_hal_async::spi::Operation;

        let a = addr & 0x7F;
        let w = [a];

        let mut ops = [Operation::Write(&w), Operation::Read(buf)];
        self.spi.transaction(&mut ops).await.map_err(Error::Spi)?;
        Ok(())
    }

    async fn write_u8(&mut self, addr: u8, data: u8) -> Result<(), Error<SPI::Error>> {
        use embedded_hal_async::spi::Operation;

        let a = addr | 0x80;
        let w = [a, data];

        let mut ops = [Operation::Write(&w)];
        self.spi.transaction(&mut ops).await.map_err(Error::Spi)?;
        Ok(())
    }
}
