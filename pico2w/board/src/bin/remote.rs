#![no_std]
#![no_main]
// #![allow(unused_imports)]

extern crate alloc;

use core::{net::Ipv4Addr, str::from_utf8, cell::RefCell};
use alloc::string::{String, ToString};
use alloc_cortex_m::CortexMHeap;

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    clocks::{RoscRng},
    gpio::{Input, Level, Output, Pull},
    peripherals::{DMA_CH0, PIO0, SPI0, SPI1},
    pio::{InterruptHandler as PioInterruptHandler, Pio},
    pwm::{Config as ConfigPwm, Pwm, SetDutyCycle},
    spi::{Async, Blocking, Config as ConfigSpi, Spi},
};
use embassy_time::{Duration, Timer, Delay};
use embassy_sync::blocking_mutex::{
    raw::{NoopRawMutex, ThreadModeRawMutex},
    NoopMutex,
};
use embassy_futures::{join, select::{select, Either}};
use defmt::{info, unwrap, warn};

use embedded_graphics::{
    prelude::Point,
    text::Text,
    primitives::{Circle, PrimitiveStyle},
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::Rgb565,
    prelude::*,
};
use mipidsi::models::ST7735s;
use mipidsi::options::{Orientation, Rotation};

use static_cell::StaticCell;
use rand::RngCore as _;

use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;

use {defmt_rtt as _};

use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_net::{tcp::TcpSocket, Config as NetConfig, Ipv4Cidr, StackResources, StaticConfigV4};

static SPI_BUS: StaticCell<NoopMutex<RefCell<Spi<'static, SPI0, Blocking>>>> = StaticCell::new();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize the heap with 32 KB
    const HEAP_SIZE: usize = 32 * 1024; // 32 KB
    unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) };

    info!("Heap initialized with size: {} bytes", HEAP_SIZE);

    let p = embassy_rp::init(Default::default());

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
    Timer::after(Duration::from_millis(500)).await;

    let mut spiconfig1 = ConfigSpi::default();
    spiconfig1.frequency = 32_000_000;

    let miso1 = p.PIN_16;
    let mosi1 = p.PIN_19;
    let clk1 = p.PIN_18;

    let mut spi1 = Spi::new_blocking(p.SPI0, clk1, mosi1, miso1, spiconfig1);
    let spi_bus = NoopMutex::new(RefCell::new(spi1));
    let spi_bus = SPI_BUS.init(spi_bus);

    let mut cs1 = Output::new(p.PIN_17, Level::High);
    let mut ad1 = Output::new(p.PIN_20, Level::Low);
    let mut reset1 = Output::new(p.PIN_21, Level::High);

    // Display initialization logic
    let spi_dev = SpiDevice::new(spi_bus, cs1);

    let mut screen_config = embassy_rp::spi::Config::default();
    screen_config.frequency = 32_000_000u32;
    screen_config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
    screen_config.polarity = embassy_rp::spi::Polarity::IdleHigh;

    let di = SPIInterface::new(spi_dev, ad1);
    let mut display = mipidsi::Builder::new(ST7735s, di)
        .reset_pin(reset1)
        .orientation(Orientation::new().rotate(Rotation::Deg270))
        .init(&mut Delay)
        .unwrap();

    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let clear_style = MonoTextStyle::new(&FONT_6X10, Rgb565::BLACK);

    display.clear(Rgb565::BLACK).unwrap();

    // Draw initial text using embedded-graphics
    Text::new("Data from Mark:", Point { x: 20, y: 20 }, text_style)
        .draw(&mut display)
        .unwrap();

    Text::new("Current LEFT: ", Point { x: 20, y: 40 }, text_style)
        .draw(&mut display)
        .unwrap();

    Text::new("Current RIGHT: ", Point { x: 20, y: 80 }, text_style)
        .draw(&mut display)
        .unwrap();

    Text::new("Mode:", Point { x: 20, y: 120 }, text_style)
        .draw(&mut display)
        .unwrap();

    let config = NetConfig::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Addr::new(169, 42, 1, 1), 24),
        dns_servers: heapless::Vec::<Ipv4Addr, 3>::new(),
        gateway: None,
    });

    let seed = RoscRng.next_u64();

    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    info!("Initializing TCP stack...");
    let (stack, runner) = embassy_net::new(_net_device, config, RESOURCES.init(StackResources::new()), seed);
    info!("TCP stack initialized.");

    info!("Spawning network task...");
    unwrap!(spawner.spawn(net_task(runner)));
    info!("Network task spawned.");

    let pico_ssid = "jones";
    let pico_password = "jack1234";

    info!("Starting WiFi access point...");
    control.start_ap_wpa2(pico_ssid, pico_password, 6).await;
    info!("WiFi access point started.");

    let mut rx_buf = [0; 4096];
    let mut tx_buf = [0; 4096];
    let mut buf = [0; 4096];

    let mut old_l: String = "aaaaaa".to_string();
    let mut old_r: String = "aaaaaa".to_string();
    let mut mode: bool = false;

    loop {
        info!("Initializing TCP socket...");
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        info!("TCP socket initialized.");

        socket.set_timeout(Some(Duration::from_secs(10)));
        info!("Socket timeout set.");

        control.gpio_set(0, false).await;
        info!("Listening on TCP: 6000");
        if let Err(e) = socket.accept(6000).await {
            warn!("Accept error: {:?}", e);
            continue;
        }

        info!("Received connection from {:?}", socket.remote_endpoint());
        control.gpio_set(0, true).await;

        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("Read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    warn!("Read error: {:?}", e);
                    break;
                }
            };

            // Copy the received data into a separate String
            let received_data = match from_utf8(&buf[..n]) {
                Ok(data) => data.trim().to_string(), // Convert to an owned String
                Err(_) => {
                    warn!("Invalid UTF-8 data received");
                    continue;
                }
            };

            // // Simulated received data for testing
            // let received_data = "LEFT:12345 RIGHT:67890 MODE:true";

            // Split the string into parts
            let mut left_data = String::new();
            let mut right_data = String::new();
            let mut mode_value = false;

            for part in received_data.split_whitespace() {
                if part.starts_with("LEFT:") {
                    left_data = part[5..].trim().to_string();
                } else if part.starts_with("RIGHT:") {
                    right_data = part[6..].trim().to_string();
                } else if part.starts_with("MODE:") {
                    mode_value = part[5..].trim().parse::<bool>().unwrap_or(false);
                }
            }

            // Update the display for LEFT
            Text::new(&old_l, Point { x: 40, y: 60 }, clear_style)
                .draw(&mut display)
                .unwrap();

            Text::new(&left_data, Point { x: 40, y: 60 }, text_style)
                .draw(&mut display)
                .unwrap();

            old_l = left_data;

            // Update the display for RIGHT
            Text::new(&old_r, Point { x: 40, y: 100 }, clear_style)
                .draw(&mut display)
                .unwrap();

            Text::new(&right_data, Point { x: 40, y: 100 }, text_style)
                .draw(&mut display)
                .unwrap();

            old_r = right_data;

            // Update the display for MODE only if it changed
            if mode != mode_value {
                Text::new(
                    if mode { "autonomous" } else { "manual" },
                    Point { x: 60, y: 120 },
                    clear_style,
                )
                .draw(&mut display)
                .unwrap();

                Text::new(
                    if mode_value { "autonomous" } else { "manual" },
                    Point { x: 60, y: 120 },
                    text_style,
                )
                .draw(&mut display)
                .unwrap();

                mode = mode_value;
            }

            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("Panic: {:?}", info);
    loop {}
}

