#![no_main]
#![no_std]

use adafruit_clue::{Board,TFT};
use cortex_m_rt;
use embedded_hal::blocking::delay::DelayMs;
use nrf52840_hal::{Timer,twim,spim,Delay};
use lsm6ds33::Lsm6ds33;
use lsm6ds33::{AccelerometerScale, AccelerometerBandwidth, AccelerometerOutput};
use lsm6ds33::{GyroscopeFullScale,GyroscopeOutput};

//use core::alloc::Layout;

use display_interface_spi::SPIInterfaceNoCS;
use st7789::{Orientation, ST7789};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::*;
use embedded_graphics::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_7X13,MonoTextStyle},
    text::Text,
};


use core::fmt::Write;
use heapless::String;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut b = Board::take().unwrap();
    let sensor_i2c_pins = twim::Pins {
        scl: b.sensors_i2c.scl,
        sda: b.sensors_i2c.sda,
    };
    let sensor_i2c = twim::Twim::new(b.TWIM1,sensor_i2c_pins,twim::Frequency::K400);
    let mut gyro_accel = Lsm6ds33::new(sensor_i2c,Board::I2C_GYROACCEL).unwrap();
    gyro_accel.set_accelerometer_scale(AccelerometerScale::G02).unwrap();
    gyro_accel.set_accelerometer_bandwidth(AccelerometerBandwidth::Freq100).unwrap();
    gyro_accel.set_accelerometer_output(AccelerometerOutput::Rate104).unwrap();
    gyro_accel.set_gyroscope_scale(GyroscopeFullScale::Dps245).unwrap();
    gyro_accel.set_gyroscope_output(GyroscopeOutput::Rate104).unwrap();
    gyro_accel.set_low_power_mode(false).unwrap();


    b.tft.backlight_on();
    // TFT SPI
    let tft_pins = spim::Pins {
        sck: b.tft.sck,
        miso: None,
        mosi: Some(b.tft.mosi),
    };
    let tft_spi = spim::Spim::new(b.SPIM0,tft_pins,spim::Frequency::M8,spim::MODE_3,122);
    let tft_display_interface = SPIInterfaceNoCS::new(tft_spi,b.tft.dc);
    let mut display = ST7789::new(tft_display_interface,b.tft.reset,TFT::XSIZE,TFT::YSIZE);
    let mut delay = Delay::new(b.core_peripherals.SYST);
    display.init(&mut delay).unwrap();
    display.set_orientation(Orientation::Landscape).unwrap();
    display.clear(Rgb565::BLACK).unwrap();
    let text_style = MonoTextStyle::new(&FONT_7X13, Rgb565::WHITE);
    let text_rect_black = Rectangle::new(Point::new(0,200), Size::new(240,40)).into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK));
    //Text::new("Hello Kitty!", Point::new(10,210),text_style).draw(&mut display).unwrap();
    let mut timer = Timer::new(b.TIMER4);
    loop {
        let (x,y,z) = gyro_accel.read_gyro().unwrap();
        let mut gyrostring: String<128> = String::new();
        write!(gyrostring,"GYRO ({:.4},{:.4},{:.4})",x,y,z).unwrap();
        let gyrotext = Text::new(&gyrostring,Point::new(10,210),text_style);
        text_rect_black.draw(&mut display).unwrap();
        gyrotext.draw(&mut display).unwrap();
        timer.delay_ms(250 as u32);
    }
}

#[panic_handler] // panicking behavior
unsafe fn panic(_pinfo: &core::panic::PanicInfo) -> ! {
    let mut b: Board = Board::steal();
    let mut timer = Timer::new(b.TIMER3);
    loop {
        b.leds.white.on();
        timer.delay_ms(500 as u32);
        b.leds.white.off();
        timer.delay_ms(100 as u32);
        //cortex_m::asm::bkpt();
    }
}
