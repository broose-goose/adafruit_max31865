#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{gpio::{Level, Output}, spi::{Config as SpiConfig, Spi}};
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use max31865::{Config, FaultCycle, Max31865Async};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    
    let miso = p.PIN_12;
    let mosi = p.PIN_11;
    let clk = p.PIN_10;
    let cs_pin = p.PIN_9;

    let spi = Spi::new(p.SPI1, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, SpiConfig::default());
    let cs = Output::new(cs_pin, Level::Low);
    let dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    let mut thermo = Max31865Async::new(dev, Config::pt1000_3wire());

    loop {
        let rtd = thermo.read_rtd(&mut Delay).await.unwrap();
        let ratio = rtd as f32 / 32768.0f32;
        info!("Ratio = {:?}", ratio);
        info!("Resistance = {:?}", ratio * thermo.config().ref_resistor.0);
        let temperature = thermo.read_temperature(&mut Delay).await.unwrap();
        info!("Temperature = {:?}", temperature);
        let fault = thermo.read_fault(FaultCycle::Auto).await.unwrap();
        if !fault.is_empty() {
            info!("Fault 0x{:02x}", fault.bits());
            for f in fault.iter() {
                info!("{}", max31865::fault_message(f));
            }
            thermo.clear_fault().await.unwrap();
        }
        Timer::after_secs(1).await;
    }
}