#![allow(clippy::unusual_byte_groupings)]
#![feature(cell_update, asm_experimental_arch)]
#![no_std]
#![no_main]
extern crate alloc;

use alloc::{boxed::Box, rc::Rc};
use core::{cell::RefCell, ops::Range, u8};

use esp_backtrace as _;
use esp_hal::{
    DriverMode,
    clock::CpuClock,
    delay::Delay,
    dma::{DmaDescriptor, descriptor_count},
    dma_buffers,
    gpio::{Flex, Level, Output},
    lcd_cam::{
        BitOrder, LcdCam,
        lcd::{
            ClockMode, Phase, Polarity,
            dpi::{self, Dpi, DpiTransfer, Format, FrameTiming},
        },
    },
    time::{Instant, Rate},
    xtensa_lx_rt::entry,
};
use log::info;
use playground::{
    display::st7701::{ManualSpi, St7701},
    dma::DmaTxStreamBuf,
};
use slint::platform::{
    Platform, WindowAdapter,
    software_renderer::{
        LineBufferProvider, MinimalSoftwareWindow, RepaintBufferType, Rgb565Pixel, TargetPixel,
    },
};
use static_cell::ConstStaticCell;

slint::include_modules!();

const RED: Rgb565Pixel = Rgb565Pixel(0b11111_000000_00000);

const V_RES: usize = 480;
const H_RES: usize = 480;
const CHUNK_LINES: usize = 16; // Number of lines per chunk
const CHUNK_PIXELS: usize = H_RES * CHUNK_LINES;
const CHUNK_BYTES: usize = CHUNK_PIXELS * 2;
const DESC_COUNT: usize = 16;

static DESCRIPTORS: ConstStaticCell<[DmaDescriptor; DESC_COUNT]> =
    ConstStaticCell::new([DmaDescriptor::EMPTY; DESC_COUNT]);
static BUFFER: ConstStaticCell<[u8; CHUNK_BYTES]> = ConstStaticCell::new([0; CHUNK_BYTES]);

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);

    dma_buffers!(100_000);

    let peripherals: esp_hal::peripherals::Peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let rst = Output::new(peripherals.GPIO47, Level::High, Default::default());
    let cs = Output::new(peripherals.GPIO21, Level::Low, Default::default());
    let scl = Output::new(peripherals.GPIO14, Level::Low, Default::default());
    let mut sda = Flex::new(peripherals.GPIO13);

    sda.set_as_output();

    let spi = ManualSpi { cs, sda, scl };

    let mut st7701 = St7701::new(spi, rst);
    let mut delay = Delay::new();

    info!("Initializing LCD");

    delay.delay_millis(50);

    st7701.init3(&mut delay).unwrap();

    info!("Initialized");

    delay.delay_millis(50);

    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
    let channel = peripherals.DMA_CH0;

    let config = dpi::Config::default()
        .with_frequency(Rate::from_mhz(12))
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftHigh,
        })
        .with_format(Format {
            enable_2byte_mode: true,
            bit_order: BitOrder::Inverted,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: H_RES,
            horizontal_total_width: 500,
            horizontal_blank_front_porch: 10,

            vertical_active_height: V_RES,
            vertical_total_height: 493,
            vertical_blank_front_porch: 2,

            hsync_width: 10,
            vsync_width: 10,

            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false);

    let dpi = Dpi::new(lcd_cam.lcd, channel, config)
        .unwrap()
        // Blue
        .with_data0(peripherals.GPIO46)
        .with_data1(peripherals.GPIO9)
        .with_data2(peripherals.GPIO10)
        .with_data3(peripherals.GPIO11)
        .with_data4(peripherals.GPIO12)
        // Green
        .with_data5(peripherals.GPIO17)
        .with_data6(peripherals.GPIO18)
        .with_data7(peripherals.GPIO8)
        .with_data8(peripherals.GPIO19)
        .with_data9(peripherals.GPIO20)
        .with_data10(peripherals.GPIO3)
        // Red
        .with_data11(peripherals.GPIO5)
        .with_data12(peripherals.GPIO6)
        .with_data13(peripherals.GPIO7)
        .with_data14(peripherals.GPIO15)
        .with_data15(peripherals.GPIO16)
        // Control
        .with_pclk(peripherals.GPIO40)
        .with_hsync(peripherals.GPIO39)
        .with_vsync(peripherals.GPIO38)
        .with_de(peripherals.GPIO37);

    let mut dma_buf = DmaTxStreamBuf::new(DESCRIPTORS.take(), BUFFER.take()).unwrap();

    let window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);

    slint::platform::set_platform(Box::new(EspBackend {
        window: window.clone(),
    }))
    .unwrap();

    window.set_size(slint::PhysicalSize::new(480, 480));
    window.show().unwrap();

    let ui = MyUI::new().unwrap();
    ui.show().unwrap();

    info!("Buffering");
    while dma_buf.push(&RED.0.to_be_bytes()) == 2 {}

    info!("Running event loop");

    let transfer = dpi.send(true, dma_buf).map_err(|e| e.0).unwrap();

    let delay = Delay::new();
    let mut buf = DmaStreamBuffer {
        inner: transfer,
        buf: [RED; H_RES],
    };

    delay.delay_millis(20);

    loop {
        info!("0");
        slint::platform::update_timers_and_animations();

        window.request_redraw();
        let dirty = window.draw_if_needed(|renderer| {
            let region = renderer.render_by_line(&mut buf);
        });

        let delay = slint::platform::duration_until_next_timer_update();
        info!("====== 114514 ======");
    }
}

struct DmaStreamBuffer<'a, Dm: DriverMode> {
    inner: DpiTransfer<'a, DmaTxStreamBuf, Dm>,
    buf: [Rgb565Pixel; H_RES],
}

impl<Dm: DriverMode> LineBufferProvider for &mut DmaStreamBuffer<'_, Dm> {
    type TargetPixel = Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        // render_fn();

        let bytes = bytemuck::cast_slice(&self.buf);
        self.inner.push(bytes, false);
        // bytemuck::write_zeroes(&mut self.buf);
    }
}

impl<Dm: DriverMode> LineBufferProvider for DmaStreamBuffer<'_, Dm> {
    type TargetPixel = Rgb565Pixel;

    fn process_line(
        mut self: &mut Self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        <&mut Self>::process_line(&mut self, line, range, render_fn);
    }
}

struct EspBackend {
    window: Rc<MinimalSoftwareWindow>,
}

impl Platform for EspBackend {
    fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(Instant::now().duration_since_epoch().as_micros())
    }

    fn debug_log(&self, arg: core::fmt::Arguments) {
        info!("Slint: {}", arg);
    }
}
