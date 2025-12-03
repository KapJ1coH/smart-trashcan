// use esp_hal::spi::master::Spi;

use defmt::info;
use display_interface::AsyncWriteOnlyDataCommand;
use display_interface_spi::SPIInterface;
use embassy_time::Timer;
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyle, MonoTextStyleBuilder, ascii::FONT_6X10, ascii::FONT_9X15_BOLD},
    // pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment},
    text::{Alignment, Text},
};

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::{delay::DelayNs, digital::Wait};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::peripherals::SPI3;
use esp_hal::spi::Mode;
use esp_hal::spi::master::Config;
use esp_hal::spi::master::Spi;
use esp_hal::time::Rate;
use panic_rtt_target as _;
use weact_studio_epd::graphics::{Display, Display290TriColor};
use weact_studio_epd::{TriColor, WeActStudio290TriColorDriver};

use core::fmt::Write;
use heapless::String;

use crate::system::{DISPLAY_DONE_SIGNAL, DISPLAY_SIGNAL, FillLevel, HUMAN_SENSOR_RESUME_SIGNAL, TRASHCAN_STATE, TrashCanState};

#[embassy_executor::task]
pub async fn display_task(
    sda: AnyPin<'static>,
    scl: AnyPin<'static>,
    cs: AnyPin<'static>,
    dc: AnyPin<'static>,
    rst: AnyPin<'static>,
    busy: AnyPin<'static>,
    spi_peripheral: SPI3<'static>,
) {
    let cs = Output::new(cs, Level::High, OutputConfig::default());
    let busy = Input::new(
        busy,
        InputConfig::default().with_pull(esp_hal::gpio::Pull::Up),
    );
    let dc = Output::new(dc, Level::Low, OutputConfig::default());
    let rst = Output::new(rst, Level::High, OutputConfig::default());

    let spi_bus = Spi::new(
        spi_peripheral,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(scl)
    .with_mosi(sda)
    .into_async();

    let spi_device =
        ExclusiveDevice::new(spi_bus, cs, embassy_time::Delay).expect("Spi device init error");
    let spi_interface = SPIInterface::new(spi_device, dc);

    let mut display_driver = StatusScreen::new(spi_interface, busy, rst, embassy_time::Delay);

    display_driver.init().await;


    loop {

        DISPLAY_SIGNAL.wait().await;
        info!("Display Signal Received");
        {

            let state = TRASHCAN_STATE.lock().await;
            info!("State Received, changing screen to {}", state.fill);
            display_driver.wake_up().await;
            display_driver.draw_state(&state);

        }

        display_driver.refresh().await;

        display_driver.sleep().await;

        DISPLAY_DONE_SIGNAL.signal(());
    }

}


/// A wrapper around the display driver to handle layout and styling.
pub struct StatusScreen<DI, BSY, RST, DLY> {
    display: Display290TriColor,
    driver: WeActStudio290TriColorDriver<DI, BSY, RST, DLY>,
    style_text_norm: MonoTextStyle<'static, TriColor>,
    style_text_bold: MonoTextStyle<'static, TriColor>,
    style_text_alert: MonoTextStyle<'static, TriColor>,

    style_border_norm: PrimitiveStyle<TriColor>,
    style_border_alert: PrimitiveStyle<TriColor>,
    style_fill_black: PrimitiveStyle<TriColor>,
    style_fill_red: PrimitiveStyle<TriColor>,
}

impl<DI, BSY, RST, DLY> StatusScreen<DI, BSY, RST, DLY>
where
    // DI: SPIInterface<
    //         ExclusiveDevice<Spi<'static, Async>, Output<'static>, embassy_time::Delay>,
    //         Output<'static>,
    //     >,
    DI: AsyncWriteOnlyDataCommand,
    BSY: InputPin + Wait,
    RST: OutputPin,
    DLY: DelayNs + Clone,
{
    pub fn new(interface: DI, busy: BSY, rst: RST, delay: DLY) -> Self {
        let display_driver = WeActStudio290TriColorDriver::new(interface, busy, rst, delay);

        let mut display = Display290TriColor::new();
        display.set_rotation(weact_studio_epd::graphics::DisplayRotation::Rotate270);

        let c_black = TriColor::Black;
        let c_red = TriColor::Red;

        // Pre-build styles for reuse
        let style_text_norm = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(c_black)
            .build();

        let style_text_bold = MonoTextStyleBuilder::new()
            .font(&FONT_9X15_BOLD)
            .text_color(c_black)
            .build();

        let style_text_alert = MonoTextStyleBuilder::new()
            .font(&FONT_9X15_BOLD)
            .text_color(c_red)
            .build();

        let style_border_norm = PrimitiveStyleBuilder::new()
            .stroke_color(c_black)
            .stroke_width(3)
            .stroke_alignment(StrokeAlignment::Inside)
            .build();

        let style_border_alert = PrimitiveStyleBuilder::new()
            .stroke_color(c_red)
            .stroke_width(4)
            .stroke_alignment(StrokeAlignment::Inside)
            .build();

        let style_fill_black = PrimitiveStyle::with_fill(c_black);
        let style_fill_red = PrimitiveStyle::with_fill(c_red);

        Self {
            display,
            driver: display_driver,
            style_text_norm,
            style_text_bold,
            style_text_alert,
            style_border_norm,
            style_border_alert,
            style_fill_black,
            style_fill_red,
        }
    }

    /// Initializes the display hardware.
    async fn init(&mut self) {
        info!("Initializing display...");
        self.driver.init().await.expect("Failed to init display");
        info!("Display initialized");
    }

    async fn refresh(&mut self) {
        self.driver
            .full_update(&self.display)
            .await
            .expect("Couldn't update the display");

    }

    fn update_bar(&mut self, state: &TrashCanState) {
        self.draw_progress_bar(state.fill);
    }

    async fn sleep(&mut self) {
        self.driver.sleep().await.expect("Sleep failed");
    }

    async fn wake_up(&mut self) {
        self.driver.wake_up().await.expect("Wake up failed");
    }

    /// Full screen refresh based on the provided state.
    pub fn draw_state(&mut self, state: &TrashCanState) {
        // self.display.clear(TriColor::White);

        let is_critical = matches!(state.fill, FillLevel::Full | FillLevel::Overflow);
        let border_style = if is_critical {
            self.style_border_alert
        } else {
            self.style_border_norm
        };

        self.display
            .bounding_box()
            .into_styled(border_style)
            .draw(&mut self.display)
            .unwrap();

        Text::new(state.id, Point::new(10, 20), self.style_text_bold)
            .draw(&mut self.display)
            .unwrap();

        Text::with_alignment(
            state.location,
            Point::new(286, 20),
            self.style_text_norm,
            Alignment::Right,
        )
        .draw(&mut self.display)
        .unwrap();

        Rectangle::new(Point::new(0, 26), Size::new(296, 2))
            .into_styled(self.style_fill_black)
            .draw(&mut self.display)
            .unwrap();

        self.draw_progress_bar(state.fill);
        self.draw_status_grid(state);
    }

    /// Renders the central fill gauge.
    fn draw_progress_bar(&mut self, level: FillLevel) {
        let bar_x = 10;
        let bar_y = 50;
        let bar_w = 276;
        let bar_h = 24;

        Text::new(
            "Fill Level:",
            Point::new(bar_x, bar_y - 6),
            self.style_text_norm,
        )
        .draw(&mut self.display)
        .unwrap();

        Rectangle::new(Point::new(bar_x, bar_y), Size::new(bar_w, bar_h))
            .into_styled(PrimitiveStyle::with_stroke(TriColor::Black, 2))
            .draw(&mut self.display)
            .unwrap();

        let percent = match level {
            FillLevel::Empty => 0.0,
            FillLevel::Quarter => 0.25,
            FillLevel::Half => 0.50,
            FillLevel::ThreeQuarters => 0.75,
            FillLevel::Full => 1.0,
            FillLevel::Overflow => 1.0,
        };

        let fill_w = (bar_w as f32 * percent) as u32;

        if fill_w > 0 {
            // Use red ink for 100% or overflow states
            let fill_style = if percent >= 1.0 {
                self.style_fill_red
            } else {
                self.style_fill_black
            };

            Rectangle::new(
                Point::new(bar_x + 2, bar_y + 2),
                Size::new(fill_w - 4, bar_h - 4),
            )
            .into_styled(fill_style)
            .draw(&mut self.display)
            .unwrap();
        }
    }

    /// Renders the bottom metadata grid (Sensors, LoRa, Alerts).
    fn draw_status_grid(&mut self, state: &TrashCanState) {
        let start_y = 95;
        let col_1 = 10;
        let col_2 = 120;
        let col_3 = 286;

        let mut text_buffer: String<128> = String::new();
        // Format: "RSSI: -80 | Payload: A1 B2..."
        write!(text_buffer, "LoRa: {} dBm", state.lora_rssi).unwrap();
        Text::new(
            &text_buffer,
            Point::new(col_1, start_y),
            self.style_text_norm,
        )
        .draw(&mut self.display)
        .unwrap();
        text_buffer.clear();

        let sens_text = if state.sensor_ok {
            "Sensors: OK"
        } else {
            "Sensors: ERR"
        };
        Text::new(
            sens_text,
            Point::new(col_1, start_y + 15),
            self.style_text_norm,
        )
        .draw(&mut self.display)
        .unwrap();

        write!(text_buffer, "Last: {}", state.last_change).unwrap();
        Text::new(
            &text_buffer,
            Point::new(col_2, start_y),
            self.style_text_norm,
        )
        .draw(&mut self.display)
        .unwrap();

        if state.raccoon_detected {
            Text::with_alignment(
                "RACCOON",
                Point::new(col_3, start_y),
                self.style_text_alert,
                Alignment::Right,
            )
            .draw(&mut self.display)
            .unwrap();

            Text::with_alignment(
                "DETECTED!",
                Point::new(col_3, start_y + 15),
                self.style_text_alert,
                Alignment::Right,
            )
            .draw(&mut self.display)
            .unwrap();
        } else {
            Text::with_alignment(
                "No Pests",
                Point::new(col_3, start_y),
                self.style_text_norm,
                Alignment::Right,
            )
            .draw(&mut self.display)
            .unwrap();
        }
    }
}
