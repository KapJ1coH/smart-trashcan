use defmt::{Format, info};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{self, Channel}, mutex::Mutex, signal::Signal};



// const DISPLAY_CHANNEL_CAPACITY: usize = 10;
// pub static DISPLAY_CHANNEL: Channel<CriticalSectionRawMutex, (), DISPLAY_CHANNEL_CAPACITY> = Channel::new();
pub static DISPLAY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub static HUMAN_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static HUMAN_SENSOR_RESUME_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static DISPLAY_DONE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub static TRASHCAN_STATE: Mutex<CriticalSectionRawMutex, TrashCanState> = Mutex::new(TrashCanState::new());



/// Represents the snapshot of the system state to be rendered.
///
/// This is intended to be passed from an Embassy task logic loop
/// to the display driver whenever an update event occurs.
#[derive(Clone, Copy, Format)]
pub struct TrashCanState<'a> {
    pub id: &'a str,
    pub location: &'a str,
    pub fill: FillLevel,
    pub lora_rssi: i16,
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
            lora_rssi: -100,
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

    loop{
        HUMAN_SIGNAL.wait().await;
        info!("Raccoon detected! Updating state and notifying display.");

        TRASHCAN_STATE.lock().await.raccoon_detected = true;

        DISPLAY_SIGNAL.signal(());
        DISPLAY_DONE_SIGNAL.wait().await;
        HUMAN_SENSOR_RESUME_SIGNAL.signal(());
    }


}
