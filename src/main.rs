#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::{Debug2Format, info};
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_graphics::Drawable;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Point;
use embedded_graphics::text::{Text, TextStyle};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::peripherals::{GPIO42, GPIO45, GPIO46, GPIO47, GPIO48, UART1};
use esp_hal::spi::Mode;
use esp_hal::spi::master::Config;
use esp_hal::spi::master::Spi;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;
use static_cell::StaticCell;
use vl53l0x::VL53L0x;
use weact_studio_epd::graphics::Display290TriColor;
use weact_studio_epd::{TriColor, WeActStudio290TriColorDriver};

use crate::display::display_task;
// use crate::lora::lora_task;
use crate::sensor::{ButtoToOpenLid, FillSensor, LidTOFSensor, Sensor, fill_level_task, human_detection_task};
use crate::servo::servo_task;
use crate::system::{DISPLAY_SIGNAL, HUMAN_SENSOR_RESUME_SIGNAL, director};

extern crate alloc;

mod display;
mod lora;
mod sensor;
mod servo;
mod system;
mod wifi;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static SPI_BUS: StaticCell<
    Mutex<CriticalSectionRawMutex, esp_hal::spi::master::Spi<'static, Async>>,
> = StaticCell::new();

type RealSensor = LidTOFSensor<I2c<'static, Async>>;

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // generator version: 1.0.1

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");


    // // Initialize SPI
    // let nss = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());
    // let sclk = peripherals.GPIO9;
    // let mosi = peripherals.GPIO10;
    // let miso = peripherals.GPIO11;

    // let reset = Output::new(peripherals.GPIO12, Level::Low, OutputConfig::default());
    // let busy = Input::new(peripherals.GPIO13, InputConfig::default());
    // let dio1 = Input::new(peripherals.GPIO14, InputConfig::default());

    // let spi = Spi::new(
    //     peripherals.SPI2,
    //     Config::default()
    //         .with_frequency(Rate::from_khz(100))
    //         .with_mode(Mode::_0),
    // )
    // .unwrap()
    // .with_sck(sclk)
    // .with_mosi(mosi)
    // .with_miso(miso)
    // .into_async();

    // let rng = esp_hal::rng::Rng::new();

    // // Initialize the static SPI bus
    // let spi_bus = SPI_BUS.init(Mutex::new(spi));

    // // spawner.spawn(lora_task(spi_bus, nss, reset, dio1, busy, rng)).ok();
    spawner.spawn(wifi::run_network(peripherals.WIFI, spawner.clone())).ok();


    let ultrasonic_rx_pin: GPIO48<'static> = peripherals.GPIO48;
    // let ultrasonic_mode_toggle_pin: GPIO47<'static> = peripherals.GPIO47;
    let uart: UART1<'static> = peripherals.UART1;

    let servo_pin = peripherals.GPIO46;
    let mcpwm_pin = peripherals.MCPWM0;

    let sda = peripherals.GPIO6;
    let scl = peripherals.GPIO5;

    let cs = peripherals.GPIO4;
    let dc = peripherals.GPIO3;
    let rst = peripherals.GPIO2;
    let busy = peripherals.GPIO1;

    spawner.spawn( display_task(sda.into(), scl.into(), cs.into(), dc.into(), rst.into(), busy.into(), peripherals.SPI3)).ok();

    let lid_scl = peripherals.GPIO40;
    let lid_sca = peripherals.GPIO41;

    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_sda(lid_sca)
    .with_scl(lid_scl)
    .into_async();

    info!("I2c initialized");

    // let mut lid_sensor = LidTOFSensor::new(i2c);
    let mut human_sensor = RealSensor::new(i2c);
    human_sensor.init().expect("Human sensor init fail");

    let mut fill_sensor = FillSensor::new(ultrasonic_rx_pin.into(),  uart);
    fill_sensor.init().expect("Failed to init fill sensor");

    let mut servo = servo::Servo::new(mcpwm_pin, servo_pin.into());

    let button_pin = Input::new(peripherals.GPIO45, InputConfig::default());

    // let button = ButtoToOpenLid::new(button_pin);

    info!("Init display refresh");
    DISPLAY_SIGNAL.signal(());
    HUMAN_SENSOR_RESUME_SIGNAL.signal(());

    spawner.spawn(human_detection_task(human_sensor)).ok();
    // spawner.spawn(fill_level_task(fill_sensor)).ok();

    spawner.spawn(director(button_pin, fill_sensor)).ok();

    servo.close();
    spawner.spawn(servo_task(servo)).ok();



}
