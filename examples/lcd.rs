#![no_main]
#![no_std]

use adafruit_clue::TFT;
use adafruit_clue::Board;
use nrf52840_hal::Timer;
use nrf52840_hal::spim;
use nrf52840_hal::Delay;
use cortex_m_rt;
use embedded_hal::blocking::delay::DelayMs;

use display_interface_spi::SPIInterfaceNoCS;
use st7789::{Orientation, ST7789};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::*;
use embedded_graphics::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10,MonoTextStyle},
    text::Text,
};

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut b = Board::take().unwrap();
    let mut timer = Timer::new(b.TIMER4);

    b.tft.backlight_on();
    //let tft_reset = port1.p1_03.into_push_pull_output(Level::Low); // reset
    //let _tft_cs = port0.p0_12.into_push_pull_output(Level::Low); // keep low while driving display
    //let tft_dc = port0.p0_13.into_push_pull_output(Level::Low); // data/clock switch
    // TFT SPI
    let tft_pins = spim::Pins {
        sck: b.tft.sck,
        miso: None,
        mosi: Some(b.tft.mosi),
    };
    let tft_spi = spim::Spim::new(b.SPIM0,tft_pins,spim::Frequency::M8,spim::MODE_3,122);
    let tft_display_interface = SPIInterfaceNoCS::new(tft_spi,b.tft.dc);
    let mut display = ST7789::new(tft_display_interface,b.tft.reset,TFT::XSIZE,TFT::YSIZE);
    let mut delay = Delay::new(b.corePeripherals.SYST);
    display.init(&mut delay).unwrap();
    display.set_orientation(Orientation::Landscape).unwrap();
    let circle1 = Circle::new(Point::new(128,64),64).into_styled(PrimitiveStyle::with_fill(Rgb565::RED));
    let circle2 = Circle::new(Point::new(64,64),64).into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN));
    let circle3 = Circle::new(Point::new(64,128),64).into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE));
    let circle4 = Circle::new(Point::new(128,128),64).into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE));
    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let howdy = Text::new("Hello Kitty!", Point::new(10,210),text_style);
    //let clear_text_rect = Rectangle::new(Point::new(0,200), Size::new(240,40)).into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK));
    display.clear(Rgb565::BLACK).unwrap();
    circle1.draw(&mut display).unwrap();
    circle2.draw(&mut display).unwrap();
    circle3.draw(&mut display).unwrap();
    circle4.draw(&mut display).unwrap();
    howdy.draw(&mut display).unwrap();

    loop {
        // Blink some LEDs
        b.leds.red.on();
        b.leds.white.off();
        timer.delay_ms(100 as u32);
        b.leds.red.off();
        b.leds.white.on();
        timer.delay_ms(100 as u32);
    }

}


#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
