use defmt::{Format, info, warn};
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{self, Channel}, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_hal::gpio::Input;
use serde::Serialize;

use crate::sensor::{FillSensor, Sensor, single_fill_level_reading};



pub static HUMAN_EVENTS: Channel<CriticalSectionRawMutex, HumanEvent, 2> = Channel::new();
pub static SERVO_EVENTS: Channel<CriticalSectionRawMutex, ServoAction, 2> = Channel::new();

pub static DISPLAY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub static DISABLE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static DISABLE_DISPLAY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static ENABLE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub static HUMAN_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static HUMAN_SENSOR_RESUME_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static DISPLAY_DONE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static CHECK_FILL_LEVEL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static NETWORK_SEND_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub static TRASHCAN_STATE: Mutex<CriticalSectionRawMutex, TrashCanState> = Mutex::new(TrashCanState::new());



#[derive(Clone, Copy, Format)]
pub enum ServoAction {
    Close,
    Open,
}

/// Events related to human (raccoon) detection
#[derive(Clone, Copy, Format)]
pub enum HumanEvent {
    Detected,
    Gone,
}


/// Represents the snapshot of the system state to be rendered.
///
/// This is intended to be passed from an Embassy task logic loop
/// to the display driver whenever an update event occurs.
#[derive(Clone, Copy, Format, Serialize)]
pub struct TrashCanState<'a> {
    pub id: &'a str,
    pub location: &'a str,
    pub fill_level: FillLevel,
    pub wifi_connected: bool,
    pub sensor_ok: bool,
    pub last_change: &'a str,
    pub raccoon_detected: bool,
}

impl<'a> TrashCanState<'a> {
    pub const fn new() -> Self {
        Self {
            id: "TC-120",
            location: "H12 Bathroom",
            fill_level: FillLevel::Empty,
            wifi_connected: false,
            sensor_ok: true,
            last_change: "now",
            raccoon_detected: false,
        }
    }

}

/// Fill level of the trash can. Used an enum for better readability.
#[derive(Clone, Copy, PartialEq, Format, Debug)]
#[repr(u8)]
pub enum FillLevel {
    Empty = 0,
    Quarter = 1,
    Half = 2,
    ThreeQuarters = 3,
    Full = 4,
    Overflow = 5,
}

impl Serialize for FillLevel {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_u8(*self as u8)
    }
}



/// Director task that manages human detection events and coordinates actions
#[embassy_executor::task]
pub async fn director(mut button: Input<'static>, mut sensor: FillSensor) {
    'director: loop {
        // Wait for a specific event (Arrived or Gone)

        // select(a, b){
        //     event = HUMAN_SIGNAL.wait,
        //     _ = button.wait_for_rising_edge()
        // }.await;

        // Used to enable button for testing and maintenance
        let result = select3(
            HUMAN_EVENTS.receiver().receive(),
            button.wait_for_rising_edge(),
            DISABLE_SIGNAL.wait(),
        ).await;

        let event;

        match result {
            Either3::First(_event) => {
                info!("Director: Human signal received.");
                event = _event;
            }
            Either3::Second(_) => {
                info!("Director: Button pressed. Simulating human arrival.");
                Timer::after(Duration::from_millis(100)).await; // Debounce delay
                SERVO_EVENTS.send(ServoAction::Open).await;

                Timer::after(Duration::from_millis(1000)).await; // Debounce delay
                
                // Loop to test the fill level while the the lid is open
                let fill_loop = async {
                    loop {
                        let level = single_fill_level_reading(&mut sensor).await.unwrap_or(FillLevel::Overflow);
                        let distance = sensor.read().await.unwrap().value;
                        info!("Trash can fill level reading: {:?}, distance: {}", level, distance);
                        {
                            let mut state = TRASHCAN_STATE.lock().await;
                            state.fill_level = level;
                        }
                    }
                };

                let wait_for_button_release = async {
                    button.wait_for_rising_edge().await;
                };
                
                select(fill_loop, wait_for_button_release).await;

                SERVO_EVENTS.send(ServoAction::Close).await;
                Timer::after(Duration::from_millis(300)).await; // Debounce delay

                continue 'director;
            }
            Either3::Third(_) => {
                info!("Director: Disabled signal received. Pausing human detection.");
                DISABLE_DISPLAY_SIGNAL.signal(());

                ENABLE_SIGNAL.wait().await;
                HUMAN_EVENTS.clear();
                info!("Director: Resuming human detection.");
                continue 'director;
            }
        }


        // let event = HUMAN_EVENTS.receive().await;
        let detected = match event {
            HumanEvent::Detected => {
                info!("Raccoon detected! Updating state.");
                let level = single_fill_level_reading(&mut sensor).await.unwrap_or(FillLevel::Overflow);
                if level == FillLevel::Full || level == FillLevel::Overflow {
                    warn!("Trash can is full or overflowing! Current level: {:?}", level);
                    continue 'director;
                }
                SERVO_EVENTS.send(ServoAction::Open).await;
                true
            },
            HumanEvent::Gone => {
                info!("Raccoon gone. Updating state. Once the lid closes in 5 sec");

                for i in (1..=5).rev() {
                    info!("Closing lid in {} seconds...", i);
                    Timer::after(Duration::from_secs(1)).await;
                }

                SERVO_EVENTS.send(ServoAction::Close).await;
                HUMAN_EVENTS.clear();
                CHECK_FILL_LEVEL.signal(()); 
                let level = single_fill_level_reading(&mut sensor).await.unwrap_or(FillLevel::Overflow);
                {
                    let mut state = TRASHCAN_STATE.lock().await;
                    state.fill_level = level;
                }
                info!("Lid closed. Fill level updated to {:?}.", level);
                // Signal the display to update only when an event actually occurred
                DISPLAY_SIGNAL.signal(());

                false
            }
        };
        {
            let mut state = TRASHCAN_STATE.lock().await;
            state.raccoon_detected = detected;
        }

        NETWORK_SEND_SIGNAL.signal(());

        Timer::after(Duration::from_secs(1)).await;
        // DISPLAY_DONE_SIGNAL.wait().await;
        HUMAN_SENSOR_RESUME_SIGNAL.signal(());
    }
}
