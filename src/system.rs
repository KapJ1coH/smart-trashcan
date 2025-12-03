use defmt::{Format, info};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{self, Channel}, mutex::Mutex, signal::Signal};



pub static HUMAN_EVENTS: Channel<CriticalSectionRawMutex, HumanEvent, 2> = Channel::new();
pub static SERVO_EVENTS: Channel<CriticalSectionRawMutex, ServoAction, 2> = Channel::new();

pub static DISPLAY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub static HUMAN_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static HUMAN_SENSOR_RESUME_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static DISPLAY_DONE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub static TRASHCAN_STATE: Mutex<CriticalSectionRawMutex, TrashCanState> = Mutex::new(TrashCanState::new());
pub static LORA_SEND_CHANNEL: Channel<CriticalSectionRawMutex, heapless::String<256>, 5> = Channel::new();



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
#[derive(Clone, Copy, Format)]
pub struct TrashCanState<'a> {
    pub id: &'a str,
    pub location: &'a str,
    pub fill: FillLevel,
    pub lora_connected: bool,
    pub sensor_ok: bool,
    pub last_change: &'a str,
    pub raccoon_detected: bool,
}

impl<'a> TrashCanState<'a> {
    pub const fn new() -> Self {
        Self {
            id: "TC-001",
            location: "H12 Bathroom",
            fill: FillLevel::Empty,
            lora_connected: false,
            sensor_ok: true,
            last_change: "N/A",
            raccoon_detected: false,
        }
    }
}

/// Fill level of the trash can. Used an enum for better readability.
#[derive(Clone, Copy, PartialEq, Format)]
pub enum FillLevel {
    Empty = 0,
    Quarter = 1,
    Half = 2,
    ThreeQuarters = 3,
    Full = 4,
    Overflow = 5,
}



#[embassy_executor::task]
pub async fn director() {
    loop {
        // Wait for a specific event (Arrived or Gone)
        let event = HUMAN_EVENTS.receive().await;
        
        {
            let mut state = TRASHCAN_STATE.lock().await;
            match event {
                HumanEvent::Detected => {
                    info!("Raccoon detected! Updating state and notifying display.");
                    state.raccoon_detected = true;
                },
                HumanEvent::Gone => {
                    info!("Raccoon gone. Updating state.");
                    state.raccoon_detected = false;
                }
            }
        }

        // Signal the display to update only when an event actually occurred
        DISPLAY_SIGNAL.signal(());
        
        DISPLAY_DONE_SIGNAL.wait().await;
        HUMAN_SENSOR_RESUME_SIGNAL.signal(());
    }
}
