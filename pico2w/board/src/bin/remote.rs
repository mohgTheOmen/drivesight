#![no_std]
#![no_main]

extern crate alloc;

use core::{cell::RefCell, net::Ipv4Addr, str::from_utf8};
use alloc::{format, string::{String, ToString}};
use alloc_cortex_m::CortexMHeap;
use libm::round;

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    clocks::{RoscRng},
    gpio::{Level, Output},
    peripherals::{DMA_CH0, PIO0, SPI0},
    pio::{InterruptHandler as PioInterruptHandler, Pio},
    spi::{Blocking, Config as ConfigSpi, Spi},
};
use embassy_time::{Duration, Timer, Delay};
use embassy_sync::blocking_mutex::{
    NoopMutex,
};
use defmt::{info, unwrap, warn};

use embedded_graphics::{
    primitives::Line,
    prelude::Point,
    text::Text,
    primitives::{Circle, PrimitiveStyle, Rectangle},
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::Rgb565,
    prelude::*,
};
use mipidsi::{models::ST7735s, options::ColorOrder};
use mipidsi::options::{Orientation};

use static_cell::StaticCell;
use rand::RngCore as _;

use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;

use {defmt_rtt as _, panic_probe as _};

use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_net::{tcp::TcpSocket, Config as NetConfig, Ipv4Cidr, StackResources, StaticConfigV4};

static SPI_BUS: StaticCell<NoopMutex<RefCell<Spi<'static, SPI0, Blocking>>>> = StaticCell::new();

// --- Interrupt bindings ---
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

// --- Global allocator for dynamic memory ---
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

/// Task to run the WiFi chip driver
#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

/// Task to run the network stack
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

// --- Helper functions for sensor display ---
fn sensor_lines_and_color(val: u32) -> (u8, Rgb565) {
    match val {
        0..=10 => (4, Rgb565::RED),
        11..=20 => (3, Rgb565::YELLOW),
        21..=40 => (2, Rgb565::YELLOW),
        41..=80 => (1, Rgb565::GREEN),
        _ => (0, Rgb565::BLACK),
    }
}

fn classify_load(val: &str) -> &'static str {
    let rounded = match val.parse::<f64>() {
        Ok(f) => round(f.abs()) as u32,
        Err(_) => return "",
    };
    match rounded {
        v if v > 200 => "HIGH",
        v if v > 100 => "MEDIUM",
        _ => "LOW",
    }
}

/// Main entry point for the application
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // --- Initialize the heap with 32 KB ---
    const HEAP_SIZE: usize = 32 * 1024;
    unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) };

    info!("Heap initialized with size: {} bytes", HEAP_SIZE);

    let p = embassy_rp::init(Default::default());

    // --- WiFi and display hardware initialization ---
    let firmware = include_bytes!("/home/andrei/Documents/School/Microprocessors/embassy/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("/home/andrei/Documents/School/Microprocessors/embassy/cyw43-firmware/43439A0_clm.bin");

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

    // --- SPI and display setup ---
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
    let mut led = Output::new(p.PIN_15, Level::High); // Set LED to high for maximum brightness

    // --- Display initialization logic ---
    let spi_dev = SpiDevice::new(spi_bus, cs1);

    let mut screen_config = embassy_rp::spi::Config::default();
    screen_config.frequency = 32_000_000u32;
    screen_config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
    screen_config.polarity = embassy_rp::spi::Polarity::IdleHigh;

    let di = SPIInterface::new(spi_dev, ad1);
    let mut display = mipidsi::Builder::new(ST7735s, di)
        .color_order(ColorOrder::Bgr)
        .reset_pin(reset1)
        .orientation(Orientation::new())
        .init(&mut Delay)
        .unwrap();

    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

    display.clear(Rgb565::BLACK).unwrap();

    Text::new("Data from Mark:", Point {x: 10, y: 10}, text_style)
        .draw(&mut display)
        .unwrap();

    Rectangle::new(Point { x: 50, y: 50 }, Size::new(35, 18))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
        .draw(&mut display)
        .unwrap();
    
    Circle::new(Point { x: 50, y: 63 }, 10)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
        .draw(&mut display)
        .unwrap();
    Circle::new(Point { x: 75, y: 63 }, 10)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
        .draw(&mut display)
        .unwrap();

    Text::new("Motor Load:", Point { x: 10, y: 95 }, text_style)
        .draw(&mut display)
        .unwrap();

    // --- Network stack configuration ---
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

    let mut old_lines_l = 0;
    let mut old_lines_f = 0;
    let mut old_lines_r = 0;
    let mut old_load_l: String = " ".to_string();
    let mut old_load_r: String = " ".to_string();

    // --- Main TCP server loop ---
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

        // --- Data receive and display update loop ---
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

            // --- Parse received data ---
            let received_data = match from_utf8(&buf[..n]) {
                Ok(data) => data.trim().to_string(),
                Err(_) => {
                    warn!("Invalid UTF-8 data received");
                    continue;
                }
            };

            // info!("Received data: {}", received_data.clone().as_str());

            let mut left_data = String::new();
            let mut front_data = String::new();
            let mut right_data = String::new();
            let mut c_left_data = String::new();
            let mut c_right_data = String::new();

            for part in received_data.split_whitespace() {
                if part.starts_with("LEFT:") {
                    left_data = part[5..].trim().to_string();
                } else if part.starts_with("FRONT:") {
                    front_data = part[6..].trim().to_string();                
                } else if part.starts_with("RIGHT:") {
                    right_data = part[6..].trim().to_string();
                } else if part.starts_with("C_LEFT:") {
                    c_left_data = part[7..].trim().to_string();
                } else if part.starts_with("C_RIGHT:") {
                    c_right_data = part[8..].trim().to_string();
                } else {
                    warn!("Unknown data format: {}", part);
                }
            }

            // --- Draw parking sensor lines for all three sensors ---

            // Parse values
            let left_val = left_data.parse::<u32>().unwrap_or(100);
            let front_val = front_data.parse::<u32>().unwrap_or(100);
            let right_val = right_data.parse::<u32>().unwrap_or(100);

            // --- LEFT SENSOR LINES ---
            let (num_lines_left, color_left) = sensor_lines_and_color(left_val);
            // Clear old lines
            if num_lines_left < old_lines_l {
                for i in num_lines_left..old_lines_l {
                    let x = 40 - i as i32 * 7;
                    let y_start = 50 - i as i32 * 4;
                    let y_end = 65 + i as i32 * 4;
                    Line::new(Point::new(x, y_start), Point::new(x, y_end))
                        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 3))
                        .draw(&mut display)
                        .unwrap();
                }
            }
            // Draw new lines
            if num_lines_left > old_lines_l {
                for i in old_lines_l..num_lines_left {
                    let x = 40 - i as i32 * 7;
                    let y_start = 50 - i as i32 * 4;
                    let y_end = 65 + i as i32 * 4;
                    Line::new(Point::new(x, y_start), Point::new(x, y_end))
                        .into_styled(PrimitiveStyle::with_stroke(color_left, 3))
                        .draw(&mut display)
                        .unwrap();
                }
            }
            old_lines_l = num_lines_left;

            // --- FRONT SENSOR LINES  ---
            let (num_lines_front, color_front) = sensor_lines_and_color(front_val);
            // Clear old lines
            if num_lines_front < old_lines_f {
                for i in num_lines_front..old_lines_f {
                    let y = 40 - i as i32 * 7;
                    let x_start = 58 - i as i32 * 6;
                    let x_end = 75 + i as i32 * 6;
                    Line::new(Point::new(x_start, y), Point::new(x_end, y))
                        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 3))
                        .draw(&mut display)
                        .unwrap();
                }
            }
            // Draw new lines
            if num_lines_front > old_lines_f {
                for i in old_lines_f..num_lines_front {
                    let y = 40 - i as i32 * 7;
                    let x_start = 58 - i as i32 * 6;
                    let x_end = 75 + i as i32 * 6;
                    Line::new(Point::new(x_start, y), Point::new(x_end, y))
                        .into_styled(PrimitiveStyle::with_stroke(color_front, 3))
                        .draw(&mut display)
                        .unwrap();
                }
            }
            old_lines_f = num_lines_front;

            // --- RIGHT SENSOR LINES ---
            let (num_lines_right, color_right) = sensor_lines_and_color(right_val);
            // Clear old lines
            if num_lines_right < old_lines_r {
                for i in num_lines_right..old_lines_r {
                    let x = 94 + i as i32 * 7;
                    let y_start = 50 - i as i32 * 4;
                    let y_end = 65 + i as i32 * 4;
                    Line::new(Point::new(x, y_start), Point::new(x, y_end))
                        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 3))
                        .draw(&mut display)
                        .unwrap();
                }
            }
            // Draw new lines
            if num_lines_right > old_lines_r {
                for i in old_lines_r..num_lines_right {
                    let x = 94 + i as i32 * 7;
                    let y_start = 50 - i as i32 * 4;
                    let y_end = 65 + i as i32 * 4;
                    Line::new(Point::new(x, y_start), Point::new(x, y_end))
                        .into_styled(PrimitiveStyle::with_stroke(color_right, 3))
                        .draw(&mut display)
                        .unwrap();
                }
            }
            old_lines_r = num_lines_right;

            // --- Parse and display current sensor values and load ---
            let c_left_val = c_left_data.parse::<f64>().unwrap_or(0.0);
            let c_right_val = c_right_data.parse::<f64>().unwrap_or(0.0);

            // Format values for display 
            let display_c_left = format!("{:.3}", c_left_val);
            let display_c_right = format!("{:.3}", c_right_val);

            // Classify load using absolute value
            let load_l = classify_load(&display_c_left).to_string();
            let load_r = classify_load(&display_c_right).to_string();

            // --- Clear and draw left load only if load changed ---
            if load_l != old_load_l {
                Rectangle::new(Point { x: 18, y: 103 }, Size::new(52, 10))
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                    .draw(&mut display)
                    .unwrap();
                Text::new(&load_l, Point { x: 20, y: 110 }, text_style)
                    .draw(&mut display)
                    .unwrap();
                old_load_l = load_l.clone();
            }

            // Clear and draw left value
            Rectangle::new(Point { x: 18, y: 113 }, Size::new(52, 10))
                .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                .draw(&mut display)
                .unwrap();
            Text::new(&display_c_left, Point { x: 20, y: 120 }, text_style)
                .draw(&mut display)
                .unwrap();

            // --- Clear and draw right load only if load changed ---
            if load_r != old_load_r {
                Rectangle::new(Point { x: 78, y: 103 }, Size::new(52, 10))
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                    .draw(&mut display)
                    .unwrap();
                Text::new(&load_r, Point { x: 80, y: 110 }, text_style)
                    .draw(&mut display)
                    .unwrap();
                old_load_r = load_r.clone();
            }

            // Clear and draw right value
            Rectangle::new(Point { x: 78, y: 113 }, Size::new(52, 10))
                .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                .draw(&mut display)
                .unwrap();
            Text::new(&display_c_right, Point { x: 80, y: 120 }, text_style)
                .draw(&mut display)
                .unwrap();

            Timer::after(Duration::from_millis(100)).await;
        }
    }
}