#![no_std]
#![no_main]

extern crate alloc;

use core::{net::Ipv4Addr, str::{self, from_utf8}, cell::RefCell, task};
use alloc::{format, string::{String, ToString}, vec::Vec};
use alloc_cortex_m::CortexMHeap;

use embassy_executor::Spawner;
use embassy_rp::{
    adc::{Adc, Channel as ChannelAdc, Config as ConfigAdc, InterruptHandler as AdcInterruptHandler}, 
    bind_interrupts, 
    clocks::{AdcClkSrc, RoscRng}, 
    gpio::{Input, Level, Output, Pull}, 
    i2c::{Config as ConfigI2c, I2c, InterruptHandler as I2cInterruptHandler}, 
    peripherals::{DMA_CH0, I2C0, I2C1, PIO0, SPI0, SPI1, UART1}, 
    pio::{InterruptHandler as PioInterruptHandler, Pio}, 
    pwm::{Config as ConfigPwm, Pwm, SetDutyCycle}, 
    spi::{Async, Blocking, Config as ConfigSpi, Spi}, 
    time_driver::init,
    uart::{Async as UartAsync, Config as UartConfig, DataBits, Parity, StopBits, Uart}
};
use embassy_time::{with_timeout, Delay, Duration, Instant, Timer};
use embassy_sync::{channel::Channel, pubsub::{PubSubChannel, WaitResult::{Lagged, Message}}};
use embassy_sync::blocking_mutex::{raw::{NoopRawMutex, ThreadModeRawMutex}, NoopMutex};
use embassy_futures::select::{select, Either};
use defmt::{info, unwrap, warn};
use embassy_embedded_hal::shared_bus::{asynch::i2c, blocking::spi::SpiDevice};
use rand::RngCore;
use static_cell::StaticCell;
use embedded_graphics::{mono_font::{ascii::FONT_6X10, MonoTextStyle}, pixelcolor::Rgb565, prelude::*, text::Text};
use {defmt_rtt as _, panic_probe as _};

use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_net::{tcp::TcpSocket, Config as NetConfig, Ipv4Cidr, StackResources, StaticConfigV4};

use itoa::{Buffer};
use hcsr04_async::{Hcsr04, Config as HcsrConfig, DistanceUnit, TemperatureUnit, Now};
// use ina219::{
//     AsyncIna219,
//     address::Address,
//     calibration::{IntCalibration, MicroAmpere},
//     configuration::*,
// };

static UART_CHANNEL: Channel<ThreadModeRawMutex, (u8, String, String, i16, i16, i16, i16, u16, u16, String), 1> = Channel::new();
static CHANNEL_FRONT: Channel<ThreadModeRawMutex, f64, 1> = Channel::new();
static CHANNEL_LEFT: Channel<ThreadModeRawMutex, f64, 1> = Channel::new();
static CHANNEL_RIGHT: Channel<ThreadModeRawMutex, f64, 1> = Channel::new();
static CHANNEL_CURRENT_L: PubSubChannel<ThreadModeRawMutex, f64, 1, 5, 1> = PubSubChannel::new();
static CHANNEL_CURRENT_R:PubSubChannel<ThreadModeRawMutex, f64, 1, 5, 1> = PubSubChannel::new();

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
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    UART1_IRQ => embassy_rp::uart::InterruptHandler<UART1>;
});

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();


#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

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
        // info!("INA219 (left) reset error: {:?}", e);
    }
    if let Err(e) = c_r.write_async(INA219_ADDRESS as u16, reset_msg).await {
        // info!("INA219 (right) reset error: {:?}", e);
    }


    let config: u16 = 0x399F;
    let config_bytes = config.to_be_bytes(); // [0x39, 0x9F]
    let config_msg = [INA219_REG_CONFIG, config_bytes[0], config_bytes[1]];
    if let Err(e) = c_l.write_async(INA219_ADDRESS as u16, config_msg).await {
        // info!("INA219 (left) config error: {:?}", e);
    }
    if let Err(e) = c_r.write_async(INA219_ADDRESS as u16, config_msg).await {
        // info!("INA219 (right) config error: {:?}", e);
    }

    let calib_l: u16 = 4096;
    let calib_l_bytes = calib_l.to_be_bytes(); // [0x10, 0x00]
    let calib_l_msg = [INA219_REG_CALIBRATION, calib_l_bytes[0], calib_l_bytes[1]];
    if let Err(e) = c_l.write_async(INA219_ADDRESS as u16, calib_l_msg).await {
        // info!("I2C (left) calibration error: {:?}", e);
    }

    let calib_r: u16 = 4096;
    let calib_r_bytes = calib_r.to_be_bytes(); // [0x10, 0x00]
    let calib_r_msg = [INA219_REG_CALIBRATION, calib_r_bytes[0], calib_r_bytes[1]];
    if let Err(e) = c_r.write_async(INA219_ADDRESS as u16, calib_r_msg).await {
        // info!("I2C (left) calibration error: {:?}", e);
    }

    Timer::after_millis(1000).await; // Wait for the sensors to configure

    let publisher_l = CHANNEL_CURRENT_L.publisher().unwrap();
    let publisher_r = CHANNEL_CURRENT_R.publisher().unwrap();

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
            // info!("I2C (left) read error: {:?}", e);
            continue;
            }
            if let Err(e) = c_r
            .write_read_async(INA219_ADDRESS as u16, [INA219_REG_CURRENT], &mut buffer_r)
            .await
            {
            // info!("I2C (right) read error: {:?}", e);
            continue;
            }

            let current_raw_l = ((buffer_l[0] as i16) << 8) | (buffer_l[1] as i16);
            let current_raw_r = ((buffer_r[0] as i16) << 8) | (buffer_r[1] as i16);

            readings_l[i] = current_raw_l as f32 * 0.1;
            readings_r[i] = current_raw_r as f32 * 0.1;

            Timer::after_millis(50).await;
        }

        let avg_l: f32 = readings_l.iter().sum::<f32>() / readings_l.len() as f32;
        let avg_r: f32 = readings_r.iter().sum::<f32>() / readings_r.len() as f32;

        publisher_l.publish(avg_l as f64).await;
        publisher_r.publish(avg_r as f64).await;
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

#[embassy_executor::task]
async fn uart_task(
    mut uart: Uart<'static, UART1, UartAsync>,
) {
    let mut buf = [0u8; 64];

    loop {
        let (_tx, mut rx) = uart.split();

        let mut buf = [0u8; 64];
        let mut index = 0;

        let mut line: Vec<u8> = Vec::new();
        
        loop {
            // Read bytes one by one
            match rx.read(&mut buf[index..=index]).await {
                Ok(_) => {
                    let byte = buf[index];
                    if byte == b'\n' {
                        if let Ok(data_str) = from_utf8(&line) {
                            let parts: Vec<&str> = data_str.trim().split(',').collect();
                            if parts.len() == 10 {
                                if let (
                                    Ok(index), dpad, buttons,
                                    Ok(axis_x), Ok(axis_y),
                                    Ok(axis_rx), Ok(axis_ry),
                                    Ok(brake), Ok(throttle),
                                    misc_buttons
                                ) = (
                                    parts[0].parse::<u8>(),
                                    parts[1].to_string(),
                                    parts[2].to_string(),
                                    parts[3].parse::<i16>(),
                                    parts[4].parse::<i16>(),
                                    parts[5].parse::<i16>(),
                                    parts[6].parse::<i16>(),
                                    parts[7].parse::<u16>(),
                                    parts[8].parse::<u16>(),
                                    parts[9].to_string(),
                                ) {
                                    UART_CHANNEL.send(
                                        (index as u8, dpad, buttons,
                                        axis_x as i16, axis_y as i16, axis_rx as i16, axis_ry as i16,
                                        brake as u16, throttle as u16, misc_buttons)
                                    ).await;

                                    let received_data = UART_CHANNEL.receive().await;
                                    info!(
                                        "Received data: ({}, {}, {}, {}, {}, {}, {}, {}, {}, {})",
                                        received_data.0,
                                        received_data.1.as_str(),
                                        received_data.2.as_str(),
                                        received_data.3,
                                        received_data.4,
                                        received_data.5,
                                        received_data.6,
                                        received_data.7,
                                        received_data.8,
                                        received_data.9.as_str()
                                    );
                                } else {
                                    info!("Parse error in one of the fields.");
                                }
                            } else {
                                info!("Unexpected number of fields: {}", data_str);
                            }
                        } else {
                            info!("UTF-8 decode error");
                        }
                        line.clear();
                    } else {
                        let _ = line.push(byte); // silently ignore overflow
                    }
    
                    index = 0;
                }
                Err(_) => {
                    Timer::after_millis(10).await;
                }
            }
        }
    }
}


#[embassy_executor::task]
async fn wifi_task(
    mut control: cyw43::Control<'static>,
    stack: embassy_net::Stack<'static>,
    wifi_ssid: &'static str,
    wifi_password: &'static str,
) {
    let mut subscriber_l = CHANNEL_CURRENT_L.subscriber().unwrap();
    let mut subscriber_r = CHANNEL_CURRENT_R.subscriber().unwrap();

    loop {
        loop {
            match control
                .join(wifi_ssid, JoinOptions::new(wifi_password.as_bytes()))
                .await
            {
                Ok(_) => {
                    info!("Wi-Fi connected.");
                    if let Some(config) = stack.config_v4() {
                        info!("IP Address: {}", config.address.address());
                    } 

                    break;
                }
                Err(err) => {
                    warn!("Wi-Fi join failed: status={}", err.status);
                    Timer::after_secs(5).await;
                    continue;
                }
            }
        }

        let mut rx_buf = [0; 4096];
        let mut tx_buf = [0; 4096];

        loop {
            let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
            socket.set_timeout(Some(Duration::from_secs(10)));
    
            let c_address = Ipv4Addr::new(169, 42, 1, 1);
            let c_port = 6000;
    
            control.gpio_set(0, false).await;
            info!("Connecting to {} via port {}", c_address, c_port);
    
            match with_timeout(Duration::from_secs(10), socket.connect((c_address, c_port))).await {
                Ok(Ok(_)) => {
                    info!("Connected to {} via port {}", c_address, c_port);
                    control.gpio_set(0, true).await;
    
                    let mut auto: bool = false;
    
                    loop {
                        let current_left = match subscriber_l.next_message().await {
                            Message(msg) => msg,
                            Lagged(_) => 0.0
                        };
                        let current_right = match subscriber_r.next_message().await {
                            Message(msg) => msg,
                            Lagged(_) => 0.0
                        };
                        
                        let msg = format!(
                            "LEFT:{:.3} RIGHT:{:.3} MODE:{}\n",
                            current_left, current_right, !auto
                        );
                        if let Err(e) = socket.write(msg.as_bytes()).await {
                            warn!("Socket write failed: {:?}", e);
                            break; 
                        }
                
                        auto = !auto;
                        Timer::after_millis(100).await;
                    }
                }
                Ok(Err(e)) => {
                    warn!("Connect error: {:?}", e);
                }
                Err(_) => {
                    warn!("TCP connect timeout after 10s");
                }
            }

            control.gpio_set(0, false).await;
            warn!("Connection lost or failed. Restarting...");
            let _ = control.leave().await;
            Timer::after_secs(3).await;
            break;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize the heap with 32 KB
    const HEAP_SIZE: usize = 32 * 1024; // 32 KB
    unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) };

    info!("Heap initialized with size: {} bytes", HEAP_SIZE);
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

    let mut uartconfig = UartConfig::default();
    uartconfig.baudrate = 9600;
    uartconfig.stop_bits = StopBits::STOP1;
    uartconfig.data_bits = DataBits::DataBits8;
    uartconfig.parity = Parity::ParityNone;

    let mut uart = Uart::new(p.UART1, p.PIN_4, p.PIN_5, Irqs, p.DMA_CH3, p.DMA_CH4, uartconfig);

    spawner.spawn(distance(trig_front, echo_front, trig_left, echo_left, trig_right, echo_right)).unwrap();
    spawner.spawn(motors(motor_l_pwm, motor_r_pwm, motor_l_in1, motor_l_in2, motor_r_in1, motor_r_in2)).unwrap();
    spawner.spawn(current(current_l, current_r)).unwrap();
    spawner.spawn(uart_task(uart)).unwrap();

    let firmware = include_bytes!("/home/andrei/Documents/School/Microprocessors/embassy/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("/home/andrei/Documents/School/Microprocessors/embassy/cyw43-firmware/43439A0_clm.bin");

    // Initialize the WiFi module
    let power = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);

    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0, 
        DEFAULT_CLOCK_DIVIDER, 
        pio.irq0, 
        cs, 
        p.PIN_24, 
        p.PIN_29, 
        p.DMA_CH0,
    );

    info!("Initializing WiFi module");

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, power, spi, firmware).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::Performance)
        .await;

    info!("WiFi module initialized");

    let config = NetConfig::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Addr::new(169, 42, 1, 2), 24),
        dns_servers: heapless::Vec::from_slice(&[Ipv4Addr::new(255, 255, 255, 0)]).unwrap(),
        gateway: Some(Ipv4Addr::new(169, 42, 1, 1)),
    });

    let seed = RoscRng.next_u64();

    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(_net_device, config, RESOURCES.init(StackResources::new()), seed);

    unwrap!(spawner.spawn(net_task(runner)));

    let wifi_ssid = "jones";
    let wifi_password = "jack1234";
    unwrap!(spawner.spawn(wifi_task(control, stack, wifi_ssid, wifi_password)));
}
