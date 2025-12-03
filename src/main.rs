#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_graphics::Drawable;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Point;
use embedded_graphics::text::{Text, TextStyle};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::peripherals::{GPIO42, GPIO45, GPIO46, GPIO47, GPIO48, UART1};
use esp_hal::spi::Mode;
use esp_hal::spi::master::Config;
use esp_hal::spi::master::Spi;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;
use weact_studio_epd::{TriColor, WeActStudio290TriColorDriver};
use weact_studio_epd::graphics::Display290TriColor;

use crate::display::display_task;
use crate::sensor::{ButtoToOpenLid, FillSensor, Sensor, human_detection_task};
use crate::system::{HUMAN_SENSOR_RESUME_SIGNAL, director};

extern crate alloc;

mod display;
mod lora;
mod sensor;
mod servo;
mod system;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

type RealSensor = ButtoToOpenLid;

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    // TODO: Spawn some tasks
    let _ = spawner;

    // let ultrasonic_rx_pin: GPIO48<'static> = peripherals.GPIO48;
    // let ultrasonic_mode_toggle_pin: GPIO47<'static> = peripherals.GPIO47;
    // let uart: UART1<'static> = peripherals.UART1;

    let servo_pin = peripherals.GPIO46;
    let mcpwm_pin = peripherals.MCPWM0;

    let sda = peripherals.GPIO6;
    let scl = peripherals.GPIO5;

    let cs = peripherals.GPIO4;
    let dc = peripherals.GPIO3;
    let rst = peripherals.GPIO2;
    let busy = peripherals.GPIO1;

    spawner.spawn( display_task(sda.into(), scl.into(), cs.into(), dc.into(), rst.into(), busy.into(), peripherals.SPI3)).ok();


    let button = Input::new(peripherals.GPIO45, InputConfig::default());

    let mut servo = servo::Servo::new(mcpwm_pin, servo_pin.into());
    let mut human_sensor = RealSensor::new(button);
    human_sensor.init().ok();
    HUMAN_SENSOR_RESUME_SIGNAL.signal(());

    spawner.spawn(human_detection_task(human_sensor)).ok();

    spawner.spawn(director()).ok();

    servo.close();
    // servo.open();
    // servo.close();

    loop {
        // let res = human_sensor.read().await.unwrap_or_default();

        // // info!("Sensor read: {}", res.value);
        // let res = human_sensor.process(res).unwrap_or(false);

        // if res {
        //     servo.open();
        // } else {
        //     servo.close();
        // }

        Timer::after_secs(1).await;
    }

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

    // // Initialize the static SPI bus
    // let spi_bus = SPI_BUS.init(Mutex::new(spi));
    // let spi_device = SpiDevice::new(spi_bus, nss);

    // // Create the SX1262 configuration
    // let sx126x_config = sx126x::Config {
    //     chip: Sx1262,
    //     tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
    //     use_dcdc: false,
    //     rx_boost: true,
    // };

    // // Create the radio instance
    // let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();
    // let mut lora = LoRa::new(
    //     Sx126x::new(spi_device, iv, sx126x_config),
    //     true,
    //     embassy_time::Delay,
    // )
    // .await
    // .unwrap();

    // let radio: LorawanRadio<_, _, MAX_TX_POWER> = lora.into();
    // let region = region::Configuration::new(LORAWAN_REGION);
    // let mut device: Device<_, _, _> = Device::new(region, radio, EmbassyTimer::new(), rng);

    // info!("Starting...");

    // loop {
    //     let response = device
    //         .join(&JoinMode::OTAA {
    //             deveui: DevEui::from([0xA7, 0x45, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70]),
    //             appkey: AppKey::from([
    //                 0xD5, 0x57, 0x81, 0x4A, 0xA9, 0x1B, 0x69, 0xAA, 0xDF, 0xA8, 0x28, 0x9C, 0x64,
    //                 0x77, 0x72, 0x9C,
    //             ]),
    //             appeui: AppEui::from([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
    //         })
    //         .await;

    //     match response {
    //         Ok(response) => match response {
    //             lorawan_device::async_device::JoinResponse::JoinSuccess => {
    //                 info!("LoRaWAN network joined succesfully!");
    //                 break;
    //             }
    //             lorawan_device::async_device::JoinResponse::NoJoinAccept => {
    //                 error!("No join accept from LoRaWAN network");
    //             }
    //         },
    //         Err(err) => {
    //             error!("{}", err);
    //             continue;
    //         }
    //     };

    //     Timer::after_millis(500).await;
    // }

    // let mut msg_buf = heapless::String::<256>::new();
    // use core::fmt::Write;

    // // Loop that waits for Web signals and activates hardware accordingly
    // loop {
    //     msg_buf.clear();
    //     let id = rng.random();
    //     let timestamp = Instant::now().as_millis();
    //     match HW_SIGNAL.wait().await {
    //         DeviceCommand::Unlock => {
    //             let distance = ultrasonic_sensor
    //                 .read()
    //                 .await
    //                 .unwrap_or(Measurement { value: 1000 });
    //             info!("distance: {}", distance.value);
    //             let res = ultrasonic_sensor.process(distance).unwrap_or(false);
    //             if res {
    //                 info!("Person is detected!");
    //                 let _ = write!(
    //                     msg_buf,
    //                     "Opening the gate. Timestamp: {}ms, ID: {}",
    //                     timestamp, id
    //                 );
    //                 green_led.set_high();
    //                 red_led.set_low();
    //                 info!("1500");
    //                 pwm_pin.set_timestamp(1500);
    //                 Timer::after_millis(1500).await;
    //             } else {
    //                 warn!("Auth OK but no person detected!");
    //                 let _ = write!(
    //                     msg_buf,
    //                     "Auth OK but no person detected! Timestamp: {}ms, ID: {}",
    //                     timestamp, id
    //                 );
    //                 HW_SIGNAL.signal(DeviceCommand::Deny);
    //             }
    //         }
    //         DeviceCommand::Deny => {
    //             let _ = write!(msg_buf, "Deny! Timestamp: {}ms, ID: {}", timestamp, id);
    //             red_led.set_high();
    //             green_led.set_low();
    //             warn!("Deny");
    //             pwm_pin.set_timestamp(500);
    //             Timer::after_millis(1500).await;
    //         }
    //     }

    //     device.send(msg_buf.as_bytes(), 1, true).await.unwrap();

    // }

    // loop {
    //     info!("Hello world!");
    //     Timer::after(Duration::from_secs(1)).await;
    // }
}

