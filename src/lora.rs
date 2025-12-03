use defmt::{error, info};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Input, Output};
use esp_hal::rng::Rng;
use esp_hal::spi::master::Spi;
use esp_hal::Async;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::lorawan_radio::LorawanRadio;
use lora_phy::sx126x::{self, Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::LoRa;
use lorawan_device::async_device::{Device, EmbassyTimer, JoinMode, region};
use lorawan_device::{AppEui, AppKey, DevEui};

use crate::system::{LORA_SEND_CHANNEL, TRASHCAN_STATE};

const LORAWAN_REGION: region::Region = region::Region::US915;
const MAX_TX_POWER: u8 = 14;

// These aliases simplify the struct definition by abstracting the complex generic types
type SpiBusType = Mutex<CriticalSectionRawMutex, Spi<'static, Async>>;
type RadioSpiDevice = SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, Async>, Output<'static>>;
type RadioIv = GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>;
type RadioKind = Sx126x<RadioSpiDevice, RadioIv, Sx1262>;
type LoRaRadio = LorawanRadio<RadioKind, embassy_time::Delay, MAX_TX_POWER>;
type LoRaWanDevice = Device<LoRaRadio, EmbassyTimer, Rng>;

/// Struct to encapsulate the LoRaWAN node state and logic
pub struct LoRaNode {
    device: LoRaWanDevice,
}

impl LoRaNode {
    /// Initializes the SPI device, LoRa Radio, and LoRaWAN stack
    pub async fn new(
        spi_bus: &'static SpiBusType,
        nss: Output<'static>,
        reset: Output<'static>,
        dio1: Input<'static>,
        busy: Input<'static>,
        rng: Rng,
    ) -> Result<Self, ()> {
        info!("LoRaNode: Initializing hardware...");

        let spi_device = SpiDevice::new(spi_bus, nss);

        let sx126x_config = sx126x::Config {
            chip: Sx1262,
            tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
            use_dcdc: false,
            rx_boost: true,
        };

        let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None)
            .map_err(|e| error!("Failed to create interface variant: {:?}", e))?;

        let lora = LoRa::new(
            Sx126x::new(spi_device, iv, sx126x_config),
            true,
            embassy_time::Delay,
        )
        .await
        .map_err(|e| error!("Failed to initialize LoRa PHY: {:?}", e))?;

        let radio: LorawanRadio<_, _, MAX_TX_POWER> = lora.into();
        let region = region::Configuration::new(LORAWAN_REGION);
        let device = Device::new(region, radio, EmbassyTimer::new(), rng);

        Ok(Self { device })
    }

    /// Handles the OTAA Join procedure
    pub async fn join(&mut self) {
        loop {
            // Update state to indicate no connection yet (High value = Bad/No Signal)
            {
                let mut state = TRASHCAN_STATE.lock().await;
                state.lora_connected = false;
            }

            info!("LoRaNode: Attempting to join network...");
            let join_mode = JoinMode::OTAA {
                deveui: DevEui::from([0xA7, 0x45, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70]),
                appkey: AppKey::from([
                    0xD5, 0x57, 0x81, 0x4A, 0xA9, 0x1B, 0x69, 0xAA, 0xDF, 0xA8, 0x28, 0x9C, 0x64,
                    0x77, 0x72, 0x9C,
                ]),
                appeui: AppEui::from([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            };

            match self.device.join(&join_mode).await {
                Ok(lorawan_device::async_device::JoinResponse::JoinSuccess) => {
                    info!("LoRaNode: Joined successfully!");
                    // Update state to indicate connection (Generic "Good" RSSI as driver abstracts real value)
                    {
                        let mut state = TRASHCAN_STATE.lock().await;
                        state.lora_connected = true;
                    }
                    break;
                }
                Ok(lorawan_device::async_device::JoinResponse::NoJoinAccept) => {
                    error!("LoRaNode: No join accept. Retrying in 5s...");
                }
                Err(e) => {
                    error!("LoRaNode: Join error: {}. Retrying in 5s...", e);
                }
            }
            Timer::after(Duration::from_secs(5)).await;
        }
    }

    /// Listens to the channel and transmits messages
    pub async fn process_queue(&mut self) {
        info!("LoRaNode: Ready to transmit.");
        loop {
            let msg = LORA_SEND_CHANNEL.receive().await;
            
            info!("LoRaNode: Sending message: {}", msg.as_str());

            match self.device.send(msg.as_bytes(), 1, true).await {
                Ok(_) => {
                    info!("LoRaNode: Message sent successfully.");
                    // Update state to indicate active connection
                    let mut state = TRASHCAN_STATE.lock().await;
                    state.lora_connected = true;
                },
                Err(e) => {
                    error!("LoRaNode: Error sending message: {}", e);
                    // Update state to indicate loss of connection
                    let mut state = TRASHCAN_STATE.lock().await;
                    state.lora_connected = false;
                },
            }

            // Simple duty cycle delay
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    /// Main run loop that combines join and process
    pub async fn run(&mut self) {
        self.join().await;
        self.process_queue().await;
    }
}

/// The main Embassy Task
#[embassy_executor::task]
pub async fn lora_task(
    spi_bus: &'static SpiBusType,
    nss: Output<'static>,
    reset: Output<'static>,
    dio1: Input<'static>,
    busy: Input<'static>,
    rng: Rng,
) {
    // Initialize the Node
    match LoRaNode::new(spi_bus, nss, reset, dio1, busy, rng).await {
        Ok(mut node) => {
            node.run().await;
        }
        Err(_) => {
            error!("LoRaNode: Initialization failed. Task exiting.");
        }
    }
}
