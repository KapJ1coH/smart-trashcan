use core::net::Ipv4Addr;
use defmt::{Debug2Format, error, info};
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::WIFI;
use esp_hal::rng::Rng;
use esp_radio::wifi::{self, ClientConfig, ModeConfig};
use esp_radio::wifi::{WifiController, WifiDevice, WifiStaState};
use heapless::Vec;
use nanofish::{
    DefaultHttpClient, DefaultHttpServer, HttpHandler, HttpHeader, HttpRequest, HttpResponse,
    ResponseBody, StatusCode, mime_types,
};
use static_cell::StaticCell;

use crate::system::{DISABLE_DISPLAY_SIGNAL, DISABLE_SIGNAL, DISPLAY_SIGNAL, ENABLE_SIGNAL, NETWORK_SEND_SIGNAL, TRASHCAN_STATE};

// 192.168.100.184
const SERVER_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 100, 184);
const SERVER_PORT: u16 = 3000;
const SSID: &str = "Kap";
// const SSID: &str = "Not a honey pot";
const PASSWORD: &str = "Nbvf12nbvf12";

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[embassy_executor::task]
pub async fn run_network(wifi_interface: WIFI<'static>, spawner: Spawner) {
    setup_wifi(wifi_interface, spawner).await;
}

async fn setup_wifi(wifi_interface: WIFI<'static>, spawner: Spawner) {
    static RADIO: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio_init = RADIO.init(esp_radio::init().unwrap());

    info!("Radio init");

    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(radio_init, wifi_interface, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let sta_device = _interfaces.sta;
    let sta_config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (sta_stack, sta_runner) = embassy_net::new(
        sta_device,
        sta_config,
        mk_static!(StackResources<6>, StackResources::<6>::new()),
        seed,
    );

    spawner.spawn(connection(_wifi_controller)).ok();
    spawner.spawn(net_task(sta_runner)).ok();

    spawner.spawn(web_server_task(sta_stack)).ok();


    wait_for_ip(sta_stack).await;

    let client = DefaultHttpClient::new(&sta_stack);

    Timer::after(Duration::from_secs(1)).await;
    NETWORK_SEND_SIGNAL.signal(());


    'wifi_loop: loop {
        NETWORK_SEND_SIGNAL.wait().await;

        // scope here so the Mutex lock is dropped immediately after serialization.
        let mut json_buf = [0u8; 512];
        let json_slice = {
            let state = TRASHCAN_STATE.lock().await;
            match serde_json_core::to_slice(&*state, &mut json_buf) {
                Ok(bytes_written) => &json_buf[..bytes_written],
                Err(e) => {
                    error!("Serialization failed: {:?}", Debug2Format(&e));
                    continue 'wifi_loop;
                }
            }
        };

        if let Ok(json_str) = core::str::from_utf8(json_slice) {
            info!("Sending JSON: {}", json_str);
        }

        let mut url_buf = heapless::String::<128>::new();
        use core::fmt::Write;
        let _ = write!(
            url_buf,
            "http://{}:{}/api/telemetry",
            SERVER_IP, SERVER_PORT
        );

        let headers = [
            HttpHeader::content_type(mime_types::JSON),
            HttpHeader::user_agent("Esp32/1.0"),
        ];

        let mut response_buffer = [0u8; 2048];

        match client
            .post(url_buf.as_str(), &headers, json_slice, &mut response_buffer)
            .await
        {
            Ok((response, _bytes_read)) => {
                info!("Response Status: {:?}", Debug2Format(&response.status_code));

                if response.is_success() {
                    info!("Telemetry sent successfully");
                    let mut state = TRASHCAN_STATE.lock().await;
                    state.wifi_connected = true;
                } else {
                    error!("Server returned error");
                    let mut state = TRASHCAN_STATE.lock().await;
                    state.wifi_connected = false;
                }
            }
            Err(e) => {
                error!("Failed to send request: {:?}", Debug2Format(&e));
                let mut state = TRASHCAN_STATE.lock().await;
                state.wifi_connected = false;
            }
        }

        // Prevent rapid-fire loops
        Timer::after(Duration::from_millis(100)).await;
    }
}

async fn wait_for_ip(sta_stack: Stack<'_>) {
    info!("Waiting for link up...");
    loop {
        if sta_stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting for IP address...");
    loop {
        if let Some(config) = sta_stack.config_v4() {
            let address = config.address.address();
            info!("Got IP: {}", address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("Start connection task");
    loop {
        if esp_radio::wifi::sta_state() == WifiStaState::Connected {
            // Already connected, wait for disconnect
            controller
                .wait_for_event(wifi::WifiEvent::StaDisconnected)
                .await;
            Timer::after(Duration::from_secs(5)).await;
        }

        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into())
                    .with_auth_method(wifi::AuthMethod::WpaWpa2Personal),
            );
            controller.set_config(&client_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!");
        }

        info!("About to connect...");
        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect to wifi: {:?}", Debug2Format(&e));
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn web_server_task(stack: Stack<'static>) {
    // Listen on Port 80
    let mut server = DefaultHttpServer::new(80);
    info!("Web Server listening on port 80 for commands...");
    server.serve(stack, WebCommandHandler).await;
}

struct WebCommandHandler;

impl HttpHandler for WebCommandHandler {
    async fn handle_request(
        &mut self,
        request: &HttpRequest<'_>,
    ) -> Result<HttpResponse<'_>, nanofish::Error> {
        let mut headers = Vec::new();
        let _ = headers.push(HttpHeader::new("Content-Type", "text/plain"));

        // Handle "/stop" endpoint
        if request.path == "/stop" {
            info!("Web Server: Received STOP command via HTTP!");

            // Trigger the signal to stop the system
            DISABLE_SIGNAL.signal(());

            Ok(HttpResponse {
                status_code: StatusCode::Ok,
                headers,
                body: ResponseBody::Text("System stopping..."),
            })
        } else if request.path == "/start" {
            info!("Web Server: Received START command via HTTP!");

            // Trigger the signal to start the system
            DISPLAY_SIGNAL.signal(());
            ENABLE_SIGNAL.signal(());

            Ok(HttpResponse {
                status_code: StatusCode::Ok,
                headers,
                body: ResponseBody::Text("System starting..."),
            })
        } else {
            Ok(HttpResponse {
                status_code: StatusCode::NotFound,
                headers,
                body: ResponseBody::Text("Command not found"),
            })
        }
    }
}
