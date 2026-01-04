#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{gpio::{Level, Output}, spi::{Config as SpiConfig, Phase, Spi}};
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use adafruit_max31865::{Config, FaultCycle, Max31865Async};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    
    // for some reason, needed to fully power cycle the breakout board
    //  and immediately configure it on power on to get it to work; might 
    //  be because the dollar breadboard I'm using but meh
    let mut pwr_pin = Output::new(p.PIN_16, Level::Low);
    Timer::after_millis(10).await;
    pwr_pin.set_high();

    let miso = p.PIN_12;
    let mosi = p.PIN_11;
    let clk = p.PIN_10;
    let cs_pin = p.PIN_13;
    let mut config = SpiConfig::default();
    config.phase = Phase::CaptureOnSecondTransition;

    let spi = Spi::new(p.SPI1, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, config);
    let cs = Output::new(cs_pin, Level::High);
    let dev  = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    let mut thermo = Max31865Async::new(dev, Config::pt1000_3wire());

    thermo.init().await.unwrap();

    loop {
        let rtd: u16 = thermo.read_rtd(&mut Delay).await.unwrap();
        info!("RTD = {:?}", rtd);
        let ratio = rtd as f32 / 32768.0f32;
        info!("Ratio = {:?}", ratio);
        info!("Resistance = {:?}", ratio * thermo.config().ref_resistor.0);
        let temperature = thermo.read_temperature(&mut Delay).await.unwrap();
        info!("Temperature = {:?}", temperature);
        let fault = thermo.read_fault(FaultCycle::Auto, &mut Delay).await.unwrap();
        if !fault.is_empty() {
            info!("Fault 0x{:02x}", fault.bits());
            for f in fault.iter() {
                info!("{}", adafruit_max31865::fault_message(f));
            }
            thermo.clear_fault().await.unwrap();
        }
        Timer::after_secs(1).await;
    }
}