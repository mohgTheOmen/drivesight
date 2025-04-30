#![no_std]
#![no_main]


use core::{cell::RefCell, task};

use embassy_executor::Spawner;
use embassy_rp::{
    adc::{Adc, Channel as ChannelAdc, Config as ConfigAdc, InterruptHandler as AdcInterruptHandler}, 
    bind_interrupts, clocks::AdcClkSrc, 
    gpio::{Input, Level, Output, Pull}, 
    i2c::{Config as ConfigI2c, I2c, InterruptHandler as I2cInterruptHandler}, 
    peripherals::{I2C0, I2C1, SPI0, SPI1}, 
    pwm::{Config as ConfigPwm, Pwm, SetDutyCycle}, 
    spi::{Async, Blocking, Config as ConfigSpi, Spi}, 
    time_driver::init
};
use embassy_time::{Delay, Duration, Instant, Timer};
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::{raw::{NoopRawMutex, ThreadModeRawMutex}, NoopMutex};
use embassy_futures::select::{select, Either};
use defmt::info;
use embassy_embedded_hal::shared_bus::{asynch::i2c, blocking::spi::SpiDevice};
use static_cell::StaticCell;
use embedded_graphics::{mono_font::{ascii::FONT_6X10, MonoTextStyle}, pixelcolor::Rgb565, prelude::*, text::Text};
use {defmt_rtt as _, panic_probe as _};

use itoa::{Buffer};
use hcsr04_async::{Hcsr04, Config as HcsrConfig, DistanceUnit, TemperatureUnit, Now};
// use ina219::{
//     AsyncIna219,
//     address::Address,
//     calibration::{IntCalibration, MicroAmpere},
//     configuration::*,
// };


static CHANNEL_FRONT: Channel<ThreadModeRawMutex, f64, 1> = Channel::new();
static CHANNEL_LEFT: Channel<ThreadModeRawMutex, f64, 1> = Channel::new();
static CHANNEL_RIGHT: Channel<ThreadModeRawMutex, f64, 1> = Channel::new();
static CHANNEL_CURRENT_L: Channel<ThreadModeRawMutex, f64, 1> = Channel::new();
static CHANNEL_CURRENT_R: Channel<ThreadModeRawMutex, f64, 1> = Channel::new();

const INA219_ADDRESS: u8 = 0x40; // Address for the INA219 sensor
const INA219_REG_BUS_VOLTAGE: u8 = 0x02;
const INA219_REG_SHUNT_VOLTAGE: u8 = 0x01;
const INA219_REG_CURRENT: u8 = 0x04;
const INA219_REG_CONFIG: u8 = 0x00;
const INA219_REG_CALIBRATION: u8 = 0x05;
const INA219_REG_RESET: u8 = 0x80; // Register to reset device

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    I2C1_IRQ => I2cInterruptHandler<I2C1>;
});

#[embassy_executor::task]
async fn current(
    mut c_r: I2c<'static, I2C1, embassy_rp::i2c::Async>,
    mut c_l: I2c<'static, I2C0, embassy_rp::i2c::Async>,
) {
    // // Perform an I2C scan on both I2C buses
    //     let mut buffer = [0u8; 1];
    //     for addr in 0x03u16..=0x77u16 {
    //         let result = current_l.write_async(addr as u8, [0x01]).await;
    //         if result.is_ok() {
    //             info!("Found device on I2C0 at address: 0x{:02X}", addr);
    //         }
    //         let result = current_r.write_async(addr as u8, [0x01]).await;
    //         if result.is_ok() {
    //             info!("Found device on I2C1 at address: 0x{:02X}", addr);
    //         }
    //     }

    let reset_msg = [INA219_REG_RESET, 0x00]; // Reset device
    if let Err(e) = c_l.write_async(INA219_ADDRESS as u16, reset_msg).await {
        info!("INA219 (left) reset error: {:?}", e);
    }
    if let Err(e) = c_r.write_async(INA219_ADDRESS as u16, reset_msg).await {
        info!("INA219 (right) reset error: {:?}", e);
    }


    let config: u16 = 0x399F;
    let config_bytes = config.to_be_bytes(); // [0x39, 0x9F]
    let config_msg = [INA219_REG_CONFIG, config_bytes[0], config_bytes[1]];
    if let Err(e) = c_l.write_async(INA219_ADDRESS as u16, config_msg).await {
        info!("INA219 (left) config error: {:?}", e);
    }
    if let Err(e) = c_r.write_async(INA219_ADDRESS as u16, config_msg).await {
        info!("INA219 (right) config error: {:?}", e);
    }

    let calib_l: u16 = 4096;
    let calib_l_bytes = calib_l.to_be_bytes(); // [0x10, 0x00]
    let calib_l_msg = [INA219_REG_CALIBRATION, calib_l_bytes[0], calib_l_bytes[1]];
    if let Err(e) = c_l.write_async(INA219_ADDRESS as u16, calib_l_msg).await {
        info!("I2C (left) calibration error: {:?}", e);
    }

    let calib_r: u16 = 4096;
    let calib_r_bytes = calib_r.to_be_bytes(); // [0x10, 0x00]
    let calib_r_msg = [INA219_REG_CALIBRATION, calib_r_bytes[0], calib_r_bytes[1]];
    if let Err(e) = c_r.write_async(INA219_ADDRESS as u16, calib_r_msg).await {
        info!("I2C (left) calibration error: {:?}", e);
    }

    Timer::after_millis(1000).await; // Wait for the sensors to configure

    loop {
        // // Read current from INA219's current register (0x04)
        // let mut buffer_l = [0u8; 2];
        // let mut buffer_r = [0u8; 2];
        // if let Err(e) = c_l
        //     .write_read_async(INA219_ADDRESS as u16, [INA219_REG_CURRENT], &mut buffer_l)
        //     .await
        // {
        //     info!("I2C (left) read error: {:?}", e);
        //     continue;
        // }
        // if let Err(e) = c_r
        //     .write_read_async(INA219_ADDRESS as u16, [INA219_REG_CURRENT], &mut buffer_r)
        //     .await
        // {
        //     info!("I2C (right) read error: {:?}", e);
        //     continue;
        // }

        // // Combine high and low byte to form the raw current value (16-bit signed integer)
        // let current_raw_l = ((buffer_l[0] as i16) << 8) | (buffer_l[1] as i16);
        // let current_l = current_raw_l as f32 * 0.1; // Convert to mA (depending on calibration)
        // let current_raw_r = ((buffer_r[0] as i16) << 8) | (buffer_r[1] as i16);
        // let current_r = current_raw_r as f32 * 0.1; // Convert to mA (depending on calibration)

        // CHANNEL_CURRENT_L.send(current_l as f64).await;
        // CHANNEL_CURRENT_R.send(current_r as f64).await;


        // let l = CHANNEL_CURRENT_L.receive().await;
        // let r = CHANNEL_CURRENT_R.receive().await;
        
        // // Log the current value
        // info!("Current (left): {} mA", l);
        // info!("Current (right): {} mA", r);

        // Timer::after_millis(100).await;

        let mut readings_l = [0.0; 20];
        let mut readings_r = [0.0; 20];

        for i in 0..20 {
            let mut buffer_l = [0u8; 2];
            let mut buffer_r = [0u8; 2];
            if let Err(e) = c_l
            .write_read_async(INA219_ADDRESS as u16, [INA219_REG_CURRENT], &mut buffer_l)
            .await
            {
            info!("I2C (left) read error: {:?}", e);
            continue;
            }
            if let Err(e) = c_r
            .write_read_async(INA219_ADDRESS as u16, [INA219_REG_CURRENT], &mut buffer_r)
            .await
            {
            info!("I2C (right) read error: {:?}", e);
            continue;
            }

            let current_raw_l = ((buffer_l[0] as i16) << 8) | (buffer_l[1] as i16);
            let current_raw_r = ((buffer_r[0] as i16) << 8) | (buffer_r[1] as i16);

            readings_l[i] = current_raw_l as f32 * 0.1;
            readings_r[i] = current_raw_r as f32 * 0.1;

            Timer::after_millis(10).await;
        }

        let avg_l: f32 = readings_l.iter().sum::<f32>() / readings_l.len() as f32;
        let avg_r: f32 = readings_r.iter().sum::<f32>() / readings_r.len() as f32;

        info!("Average Current (left): {} mA", avg_l);
        info!("Average Current (right): {} mA", avg_r);
    }
}

#[embassy_executor::task]
async fn distance(
    trig_front: Output<'static>,
    echo_front: Input<'static>,
    trig_left: Output<'static>,
    echo_left: Input<'static>,
    trig_right: Output<'static>,
    echo_right: Input<'static>,
) {
    let config_front = HcsrConfig {
        distance_unit: DistanceUnit::Centimeters,
        temperature_unit: TemperatureUnit::Celsius,
    };
    
    let config_left = HcsrConfig {
        distance_unit: DistanceUnit::Centimeters,
        temperature_unit: TemperatureUnit::Celsius,
    };
    
    let config_right = HcsrConfig {
        distance_unit: DistanceUnit::Centimeters,
        temperature_unit: TemperatureUnit::Celsius,
    };

    struct EmbassyClock;

    impl Now for EmbassyClock {
        fn now_micros(&self) -> u64 {
            Instant::now().as_micros()
        }
    }

    let mut hc_front = Hcsr04::new(trig_front, echo_front, config_front, EmbassyClock, Delay);
    let mut hc_left = Hcsr04::new(trig_left, echo_left, config_left, EmbassyClock, Delay);
    let mut hc_right = Hcsr04::new(trig_right, echo_right, config_right, EmbassyClock, Delay);

    let temperature: f64 = 24.0;

    loop {
        let distance_front = hc_front.measure(temperature).await.unwrap_or(f64::MAX);
        let distance_left = hc_left.measure(temperature).await.unwrap_or(f64::MAX);
        let distance_right = hc_right.measure(temperature).await.unwrap_or(f64::MAX);

        CHANNEL_FRONT.send(distance_front).await;
        CHANNEL_LEFT.send(distance_left).await;
        CHANNEL_RIGHT.send(distance_right).await;

        let front = CHANNEL_FRONT.receive().await;
        let left = CHANNEL_LEFT.receive().await;
        let right = CHANNEL_RIGHT.receive().await;

        // info!("Front distance: {} cm", front);
        // info!("Left distance: {} cm", left);
        // info!("Right distance: {} cm", right);

        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn motors(
    mut l_pwm: Pwm<'static>,
    mut r_pwm: Pwm<'static>,
    mut l_in1: Output<'static>,
    mut l_in2: Output<'static>,
    mut r_in1: Output<'static>,
    mut r_in2: Output<'static>,
) {
    l_pwm.set_duty_cycle(0);
    r_pwm.set_duty_cycle(0);
    l_in1.set_high();
    l_in2.set_low();
    r_in1.set_high();
    r_in2.set_low();


    loop {
        // motor.a.forward().set_duty(50.0);
        l_pwm.set_duty_cycle_percent(100);
        r_pwm.set_duty_cycle_percent(100);

        // Wait before the next read
        Timer::after_millis(1000).await;

        // // motor.a.stop();
        // l_pwm.set_duty_cycle_percent(50);
        // r_pwm.set_duty_cycle_percent(50);

        // Timer::after_millis(1000).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let motor_l_pwm = Pwm::new_output_a(p.PWM_SLICE0, p.PIN_16, ConfigPwm::default());
    let motor_r_pwm = Pwm::new_output_a(p.PWM_SLICE1, p.PIN_18, ConfigPwm::default());
    let motor_l_in1 = Output::new(p.PIN_17, Level::Low);
    let motor_l_in2 = Output::new(p.PIN_20, Level::Low);
    let motor_r_in1 = Output::new(p.PIN_19, Level::Low);
    let motor_r_in2 = Output::new(p.PIN_21, Level::Low);

    let trig_front = Output::new(p.PIN_12, Level::Low);
    let echo_front = Input::new(p.PIN_13, Pull::Down);
    let trig_left = Output::new(p.PIN_14, Level::Low);
    let echo_left = Input::new(p.PIN_15, Pull::Down);
    let trig_right = Output::new(p.PIN_10, Level::Low);
    let echo_right = Input::new(p.PIN_11, Pull::Down);

    let mut current_l = I2c::new_async(p.I2C1, p.PIN_7, p.PIN_6, Irqs, ConfigI2c::default());
    let mut current_r = I2c::new_async(p.I2C0, p.PIN_9, p.PIN_8, Irqs, ConfigI2c::default());

    spawner.spawn(distance(trig_front, echo_front, trig_left, echo_left, trig_right, echo_right)).unwrap();
    spawner.spawn(motors(motor_l_pwm, motor_r_pwm, motor_l_in1, motor_l_in2, motor_r_in1, motor_r_in2)).unwrap();
    spawner.spawn(current(current_l, current_r)).unwrap();
}
