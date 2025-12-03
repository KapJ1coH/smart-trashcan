use defmt::{Debug2Format, Format, error, info};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use esp_hal::peripherals::I2C0;
use esp_hal::time::Rate;
use esp_hal::uart::{Config as UARTConfig, Uart};
use esp_hal::{
    Blocking,
    gpio::{AnyPin, Input, Output},
    peripherals::{GPIO6, UART1},
};

use esp_hal::i2c::master::Config as I2cConfig;
use vl53l0x::VL53L0x;

use crate::system::{HUMAN_EVENTS, HUMAN_SENSOR_RESUME_SIGNAL, HUMAN_SIGNAL, HumanEvent};
use crate::{RealSensor, sensor};

const READ_TOTAL_TIME: Duration = Duration::from_millis(1000);
const READ_TIMEOUT_BETWEEN_READS: Duration = Duration::from_millis(100);
const HEADER_BYTE: u8 = 0xFF;

const SENSOR_SLEEP: Duration = Duration::from_millis(10);

#[embassy_executor::task]
pub async fn human_detection_task(mut sensor: RealSensor) {
    let mut was_detected = false;
    loop {
        let measurement = sensor.read().await;
        match measurement {
            Ok(meas) => {
                let is_detected = sensor.process(meas);
                match is_detected {
                    Ok(is_detected) => {
                        // Only send event if state changed
                        if is_detected && !was_detected {
                            info!("Human detected!");
                            HUMAN_EVENTS.send(HumanEvent::Detected).await;
                            was_detected = true;
                        } else if !is_detected && was_detected {
                            info!("Human left!");
                            HUMAN_EVENTS.send(HumanEvent::Gone).await;
                            was_detected = false;
                        }
                    }
                    Err(e) => {
                        error!("Error processing measurement: {:?}", e);
                    }
                }
            }
            Err(e) => {
                error!("Error reading sensor: {:?}", e);
            }
        }

        Timer::after(SENSOR_SLEEP).await;
    }
}

/// Trait defining the interface for sensors
///
/// Enables different sensor implementations to be used interchangeably in the state machine.
pub trait Sensor {
    /// initialize all the hardware
    fn init(&mut self) -> Result<(), SensorError>;
    /// Take a measurement
    async fn read(&mut self) -> Result<Measurement, SensorError>;
    /// In case power needs to be cut off
    fn sleep(&mut self) -> Result<(), SensorError>;
    /// Process the value and output detection status
    fn process(&mut self, meas: Measurement) -> Result<bool, SensorError>;
}

/// Error types for sensor operations
#[derive(Debug, Format)]
pub enum SensorError {
    Bus,
    Timeout,
    BadData,
}

/// Struct to hold measurement data
///
/// More might be added later
#[derive(Debug, Clone, Copy, Format, Default)]
pub struct Measurement {
    pub value: u16,
}

pub struct ButtoToOpenLid {
    button_pin: Input<'static>,
}

impl ButtoToOpenLid {
    pub fn new(button_pin: Input<'static>) -> Self {
        Self { button_pin }
    }
}

impl Sensor for ButtoToOpenLid {
    fn init(&mut self) -> Result<(), SensorError> {
        info!("[SENSOR STATE] - Init Over");
        Ok(())
    }

    async fn read(&mut self) -> Result<Measurement, SensorError> {
        Timer::after_millis(5).await;

        let pressed = if self.button_pin.is_high() {
            1_u16
        } else {
            0_u16
        };

        Ok(Measurement { value: pressed })
    }

    fn sleep(&mut self) -> Result<(), SensorError> {
        info!("[SENSOR STATE] - Sleep");
        Ok(())
    }

    fn process(&mut self, meas: Measurement) -> Result<bool, SensorError> {
        if meas.value == 1 { Ok(true) } else { Ok(false) }
    }
}

pub struct LidTOFSensor<I2C>
where
    I2C: I2c,
{
    // sensor: Hcsr04<Output<'static>, Input<'static>, Delay>
    sensor: VL53L0x<I2C>,
}

impl<I2C> Sensor for LidTOFSensor<I2C>
where
    I2C: I2c,
{
    fn init(&mut self) -> Result<(), SensorError> {
        if let Err(e) = self.sensor.set_measurement_timing_budget(20_000) {
            info!("Error setting timing budget: {:?}", Debug2Format(&e));
        }

        // Start Continuous Mode (0 = run forever)
        if let Err(e) = self.sensor.start_continuous(0) {
            info!("Error starting continuous mode: {:?}", Debug2Format(&e));
        }
        info!("VL53L0x initialized");
        Ok(())
    }

    async fn read(&mut self) -> Result<Measurement, SensorError> {
        self.sensor
            .read_range_continuous_millimeters_blocking()
            .map(|distance| Measurement { value: distance })
            .map_err(|e| {
                info!("Error reading range: {:?}", Debug2Format(&e));
                SensorError::Bus
            })


    }

    fn sleep(&mut self) -> Result<(), SensorError> {
        todo!()
    }

    fn process(&mut self, meas: Measurement) -> Result<bool, SensorError> {
        match meas.value {
            distance if distance < 500 => Ok(true),
            _ => Ok(false),                       
        }
    }
}

impl<I2C> LidTOFSensor<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C) -> Self {

        let sensor = VL53L0x::new(i2c).expect("Failed to init VL53L0x");
        Self { sensor }
    }
}


/// Ultrasonic sensor implementation
pub struct FillSensor {
    uart: Uart<'static, Blocking>,
    ultrasonic_mode_toggle_pin: Output<'static>,
}

/// Implementation for UltrasonicSensor
impl FillSensor {
    /// Constructor for UltrasonicSensor
    pub fn new(
        ultrasonic_rx_pin: AnyPin<'static>,
        ultrasonic_mode_toggle_pin: Output<'static>,
        uart: UART1<'static>,
    ) -> Self {
        let uart_cfg = UARTConfig::default().with_baudrate(9_600);
        // let ultra_tx = Output::new(
        //     ultrasonic_mode_toggle_pin,
        //     esp_hal::gpio::Level::High,
        //     OutputConfig::default(),
        // );
        Self {
            uart: Uart::new(uart, uart_cfg)
                .unwrap()
                .with_rx(ultrasonic_rx_pin),
            ultrasonic_mode_toggle_pin,
        }
    }
}

impl Sensor for FillSensor {
    /// initialize all the hardware
    fn init(&mut self) -> Result<(), SensorError> {
        self.ultrasonic_mode_toggle_pin.set_high();
        info!("[SENSOR STATE] - Init Over");
        Ok(())
    }
    /// Take a measurement
    async fn read(&mut self) -> Result<Measurement, SensorError> {
        info!("[SENSOR STATE] - Read");
        let mut last_value: u16 = 0;
        let start = Instant::now();

        while Instant::now().duration_since(start) < READ_TOTAL_TIME {
            Timer::after(READ_TIMEOUT_BETWEEN_READS).await;
            let mut buf = [0u8; 4];
            if self.uart.read(&mut buf).is_ok() && buf[0] == HEADER_BYTE {
                let checksum = buf[0] + buf[1] + buf[2];
                if checksum != buf[3] {
                    continue;
                }
                last_value = (buf[1] as u16) << 8 | (buf[2] as u16);
            }
        }
        Ok(Measurement { value: last_value })
    }
    /// In case power needs to be cut off
    fn sleep(&mut self) -> Result<(), SensorError> {
        info!("[SENSOR STATE] - Sleep");
        Ok(())
    }

    /// Process the value and output detection status
    fn process(&mut self, meas: Measurement) -> Result<bool, SensorError> {
        // TODO return false if full, else true
        if meas.value < 600 && meas.value != 0 {
            Ok(true)
        } else {
            Ok(false)
        }
    }
}
