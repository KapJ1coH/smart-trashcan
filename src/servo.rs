use defmt::info;
use esp_hal::gpio::AnyPin;
use esp_hal::mcpwm::operator::{PwmPin, PwmPinConfig};
use esp_hal::mcpwm::timer::PwmWorkingMode;
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig};
use esp_hal::peripherals::MCPWM0;
use esp_hal::time::Rate;

use crate::system::SERVO_EVENTS;



#[embassy_executor::task]
pub async fn servo_task(mut servo: Servo) {
    loop {
        let action = SERVO_EVENTS.receiver().receive().await;

        match action {
            crate::system::ServoAction::Open => {
                info!("Servo task: Open command received");
                servo.open();
            }
            crate::system::ServoAction::Close => {
                info!("Servo task: Close command received");
                servo.close();
            }
        }
    }
}


/// Struct representing a servo motor controlled via MCPWM
///
/// Example usage:
///
/// ```rust
///     servo.open();
///     servo.close();
///     let res = servo.is_open();
///
/// ```
///
pub struct Servo {
    servo_data_pin: PwmPin<'static, MCPWM0<'static>, 0, true>,
    min_pulse: u16,
    max_pulse: u16,
}

/// Servo implementation
impl Servo {
    pub fn new(mcpwm_pin: MCPWM0<'static>, servo_pin: AnyPin<'static>) -> Self {
        let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
        let mut mcpwm = McPwm::new(mcpwm_pin, clock_cfg);

        mcpwm.operator0.set_timer(&mcpwm.timer0);
        let pwm_pin = mcpwm
            .operator0
            .with_pin_a(servo_pin, PwmPinConfig::UP_ACTIVE_HIGH);

        let timer_clock_cfg = clock_cfg
            .timer_clock_with_frequency(19_999, PwmWorkingMode::Increase, Rate::from_hz(50))
            .unwrap();
        mcpwm.timer0.start(timer_clock_cfg);

        Servo {
            servo_data_pin: pwm_pin,
            min_pulse: 2000,
            max_pulse: 2400,
        }
    }

    /// Check if the servo is in the open position
    pub fn is_open(&self) -> bool {
        self.servo_data_pin.timestamp() == self.max_pulse
    }

    /// Check if the servo is in the closed position
    pub fn is_closed(&self) -> bool {
        self.servo_data_pin.timestamp() == self.min_pulse
    }

    /// Open the servo (set to max pulse width)
    pub fn open(&mut self) {
        info!("Servo open: {}", self.min_pulse);
        self.set_timestamp(self.min_pulse)
    }

    /// Open the servo (set to min pulse width)
    pub fn close(&mut self) {
        info!("Servo close: {}", self.max_pulse);
        self.set_timestamp(self.max_pulse)
    }

    /// Set the servo pulse width in microseconds
    fn set_timestamp(&mut self, value: u16) {
        self.servo_data_pin.set_timestamp(value)
    }
}
