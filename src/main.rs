//! This example test the ADC (Analog to Digital Conversion) of the RS2040 pin 26, 27 and 28.
//! It also reads the temperature sensor in the chip.

#![no_std]
#![no_main]

use core::num::Wrapping;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};
use embassy_rp::gpio;
use embassy_rp::gpio::Pull;
use embassy_rp::bind_interrupts;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => InterruptHandler;
});

// this feels clumsy? Can i create a single mutex for the whole bus?
type BusOutType = Mutex<ThreadModeRawMutex, Option<Output<'static>>>;
static BUS_OUT: [BusOutType; 8] = [
    Mutex::new(None),
    Mutex::new(None),
    Mutex::new(None),
    Mutex::new(None),
    Mutex::new(None),
    Mutex::new(None),
    Mutex::new(None),
    Mutex::new(None),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p: embassy_rp::Peripherals = embassy_rp::init(Default::default());
    let mut adc = Adc::new(p.ADC, Irqs, Config::default());

    let mut p26 = Channel::new_pin(p.PIN_26, Pull::None);
    let mut p27 = Channel::new_pin(p.PIN_27, Pull::None);
    let mut p28 = Channel::new_pin(p.PIN_28, Pull::None);
    let mut ts = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    {
        *(BUS_OUT[0].lock().await) = Some(Output::new(p.PIN_2, Level::Low));
    }
    {
        *(BUS_OUT[1].lock().await) = Some(Output::new(p.PIN_3, Level::Low));
    }
    {
        *(BUS_OUT[2].lock().await) = Some(Output::new(p.PIN_4, Level::Low));
    }
    {
        *(BUS_OUT[3].lock().await) = Some(Output::new(p.PIN_5, Level::Low));
    }
    {
        *(BUS_OUT[4].lock().await) = Some(Output::new(p.PIN_6, Level::Low));
    }
    {
        *(BUS_OUT[5].lock().await) = Some(Output::new(p.PIN_7, Level::Low));
    }
    {
        *(BUS_OUT[6].lock().await) = Some(Output::new(p.PIN_8, Level::Low));
    }
    {
        *(BUS_OUT[7].lock().await) = Some(Output::new(p.PIN_9, Level::Low));
    }

    unwrap!(_spawner.spawn(bus_out(&BUS_OUT)));

    loop {
        let level = adc.read(&mut p26).await.unwrap();
        info!("Pin 26 ADC: {}", level);
        let level = adc.read(&mut p27).await.unwrap();
        info!("Pin 27 ADC: {}", level);
        let level = adc.read(&mut p28).await.unwrap();
        info!("Pin 28 ADC: {}", level);
        let temp = adc.read(&mut ts).await.unwrap();
        info!("Temp: {} degrees", convert_to_celsius(temp));
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn bus_out(bus: &'static [BusOutType; 8]) {
    let mut out_val = Wrapping(0u8);

    loop {
        for (index, out) in bus.iter().enumerate() {
            if (out_val.0 >> index & 0x01) != 0 {
                let mut out_unlocked = out.lock().await;
                if let Some(pin_ref) = out_unlocked.as_mut() {
                    pin_ref.set_high();
                }
            } else {
                let mut out_unlocked = out.lock().await;
            if let Some(pin_ref) = out_unlocked.as_mut() {
                pin_ref.set_low();
            }
            }
        }
        out_val += 1;
        Timer::after_secs(1).await;
    }
}

fn convert_to_celsius(raw_temp: u16) -> f32 {
    // According to chapter 4.9.5. Temperature Sensor in RP2040 datasheet
    let temp = 27.0 - (raw_temp as f32 * 3.3 / 4096.0 - 0.706) / 0.001721;
    let sign = if temp < 0.0 { -1.0 } else { 1.0 };
    let rounded_temp_x10: i16 = ((temp * 10.0) + 0.5 * sign) as i16;
    (rounded_temp_x10 as f32) / 10.0
}
