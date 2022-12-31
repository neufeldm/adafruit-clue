#![no_main]
#![no_std]

use adafruit_clue::Board;
use adafruit_clue::TFT;
use cortex_m_rt;
use embedded_hal::blocking::delay::DelayMs;
use nrf52840_hal::Delay;
use nrf52840_hal::Timer;

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    text::Text,
};
use st7789::{Orientation, ST7789};

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut b = Board::take().unwrap();
    let mut timer = Timer::new(b.TIMER4);

    // TFT SPI
    b.tft.backlight_on();
    let tft_display_interface =
        SPIInterfaceNoCS::new(b.tft.spim(b.SPIM0), b.tft.dc.take().unwrap());
    let mut display = ST7789::new(
        tft_display_interface,
        b.tft.reset.take().unwrap(),
        TFT::XSIZE,
        TFT::YSIZE,
    );
    let mut delay = Delay::new(b.core_peripherals.SYST);
    display.init(&mut delay).unwrap();
    display.set_orientation(Orientation::Landscape).unwrap();
    display.clear(Rgb565::BLACK).unwrap();
    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    Text::new("Hello Kitty!", Point::new(10, 210), text_style)
        .draw(&mut display)
        .unwrap();
    let make_circle = |x, y, color: Rgb565| {
        Circle::new(Point::new(x, y), 64).into_styled(PrimitiveStyle::with_fill(color))
    };
    let mut circle_colors = [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE, Rgb565::WHITE];
    loop {
        make_circle(128, 64, circle_colors[0])
            .draw(&mut display)
            .unwrap();
        make_circle(64, 64, circle_colors[1])
            .draw(&mut display)
            .unwrap();
        make_circle(64, 128, circle_colors[2])
            .draw(&mut display)
            .unwrap();
        make_circle(128, 128, circle_colors[3])
            .draw(&mut display)
            .unwrap();
        let last_color = circle_colors[circle_colors.len() - 1];
        for i in (1..circle_colors.len()).rev() {
            circle_colors[i] = circle_colors[i - 1];
        }
        circle_colors[0] = last_color;
        timer.delay_ms(1000 as u32);
    }
}

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
