use defmt::{Format, info};
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{self, Channel}, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_hal::gpio::Input;



pub static HUMAN_EVENTS: Channel<CriticalSectionRawMutex, HumanEvent, 2> = Channel::new();
pub static SERVO_EVENTS: Channel<CriticalSectionRawMutex, ServoAction, 2> = Channel::new();

pub static DISPLAY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub static HUMAN_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static HUMAN_SENSOR_RESUME_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static DISPLAY_DONE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static CHECK_FILL_LEVEL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

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



/// Director task that manages human detection events and coordinates actions
#[embassy_executor::task]
pub async fn director(mut button: Input<'static>) {
    'director: loop {
        // Wait for a specific event (Arrived or Gone)

        // select(a, b){
        //     event = HUMAN_SIGNAL.wait,
        //     _ = button.wait_for_rising_edge()
        // }.await;

        // Used to enable button for testing and maintenance
        let result = select(
            HUMAN_EVENTS.receiver().receive(),
            button.wait_for_rising_edge(),
        ).await;

        let event;

        match result {
            Either::First(_event) => {
                info!("Director: Human signal received.");
                event = _event;
            }
            Either::Second(_) => {
                info!("Director: Button pressed. Simulating human arrival.");
                Timer::after(Duration::from_millis(100)).await; // Debounce delay
                SERVO_EVENTS.send(ServoAction::Open).await;

                Timer::after(Duration::from_millis(1000)).await; // Debounce delay

                // Wait for another press to close the lid
                button.wait_for_rising_edge().await;
                SERVO_EVENTS.send(ServoAction::Close).await;
                Timer::after(Duration::from_millis(300)).await; // Debounce delay

                continue 'director;
            }
        }


        // let event = HUMAN_EVENTS.receive().await;
        let deteceted = match event {
            HumanEvent::Detected => {
                info!("Raccoon detected! Updating state and notifying display.");
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
                CHECK_FILL_LEVEL.signal(()); 

                false
            }
        };
        {
            let mut state = TRASHCAN_STATE.lock().await;
            state.raccoon_detected = deteceted;
        }

        // Signal the display to update only when an event actually occurred
        DISPLAY_SIGNAL.signal(());
        
        DISPLAY_DONE_SIGNAL.wait().await;
        HUMAN_SENSOR_RESUME_SIGNAL.signal(());
    }
}
