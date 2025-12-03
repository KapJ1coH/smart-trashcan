use defmt::{Format, error, info};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal::delay::DelayNs;
use esp_hal::uart::{Config as UARTConfig, Uart};
use esp_hal::{
    Blocking,
    gpio::{AnyPin, Input, Output},
    peripherals::{GPIO6, UART1},
};
use hcsr04::{Hcsr04, NoTemperatureCompensation};

use crate::system::{HUMAN_SENSOR_RESUME_SIGNAL, HUMAN_SIGNAL};
use crate::{RealSensor, sensor};

const READ_TOTAL_TIME: Duration = Duration::from_millis(1000);
const READ_TIMEOUT_BETWEEN_READS: Duration = Duration::from_millis(100);
const HEADER_BYTE: u8 = 0xFF;

const SENSOR_SLEEP: Duration = Duration::from_millis(100);

#[embassy_executor::task]
pub async fn human_detection_task(mut sensor: RealSensor) {
    'detection_loop: loop {
        if !HUMAN_SENSOR_RESUME_SIGNAL.signaled() {
            continue 'detection_loop;
        }

        let measurement = sensor.read().await;
        match measurement {
            Ok(meas) => {
                let detected = sensor.process(meas);
                match detected {
                    Ok(status) => {
                        if status {
                            info!("Human detected!");
                            HUMAN_SENSOR_RESUME_SIGNAL.reset();
                            HUMAN_SIGNAL.signal(());
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

/// Lid ultrasonic sensor implementation, specific to human detection
pub struct LidUltrasonicSensor {
    // sensor: Hcsr04<Output<'static>, Input<'static>, Delay>
    trig: Output<'static>,
    echo: Input<'static>,
}

impl LidUltrasonicSensor {
    pub fn new(trig: Output<'static>, echo: Input<'static>) -> Self {
        Self {
            // sensor: Hcsr04::builder().trig(trig).echo(echo).delay(Delay).temperature(NoTemperatureCompensation).build()
            trig,
            echo,
        }
    }
}

/// Trait implementation for LidUltrasonicSensor
impl Sensor for LidUltrasonicSensor {
    fn init(&mut self) -> Result<(), SensorError> {
        info!("[SENSOR STATE] - Init Over");
        Ok(())
    }

    /// Read distance measurement from the sensor
    async fn read(&mut self) -> Result<Measurement, SensorError> {
        // let distance: f32 = self.sensor.measure_distance().await.unwrap_or(0.0);
        Timer::after_millis(5).await;

        self.trig.set_low();
        Timer::after_micros(2).await;
        self.trig.set_high();
        Timer::after_micros(10).await;
        self.trig.set_low();

        // Wait for rising edge with timeout
        let start = Instant::now();
        while self.echo.is_low() {
            if (Instant::now() - start).as_millis() > 25 {
                return Err(SensorError::Timeout);
            }
        }
        let time1 = Instant::now();
        let mut counter = 0_u128;
        while self.echo.is_high() {
            if (Instant::now() - time1).as_millis() > 25 {
                error!("Timeout");
                return Err(SensorError::Timeout);
            }

            counter += 1;
        }

        let time2 = Instant::now();
        let pulse_width = (time2 - time1).as_micros() as f32;

        let distance = ((pulse_width * 0.343) / 2_f32) as u16;

        info!("pulse_width: {}, distance: {}", pulse_width, distance);
        info!("counter: {}", counter);

        Ok(Measurement { value: distance })
    }

    fn sleep(&mut self) -> Result<(), SensorError> {
        info!("[SENSOR STATE] - Sleep");
        Ok(())
    }

    /// Process the measurement to determine if a human is detected
    fn process(&mut self, meas: Measurement) -> Result<bool, SensorError> {
        // Todo calibrate

        if meas.value > 300 {
            Ok(false)
        } else {
            Ok(true)
        }
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
