#![no_main]
#![no_std]

use adafruit_clue::prelude::OutputPin;
use adafruit_clue::{Board, ButtonUpDown, Microphone, TFT};
use core::convert::TryInto;
use cortex_m_rt;
use embedded_hal::blocking::delay::DelayMs;
use nrf52840_hal::clocks::Clocks;
use nrf52840_hal::{spim, Delay, Timer};

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use st7789::{Orientation, ST7789};

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut b = Board::take().unwrap();
    b.tft.backlight.set_high().unwrap();
    // TFT SPI
    let tft_spim_pins = adafruit_clue::tft_spim_pins(b.tft.sck, b.tft.mosi);
    let tft_spi = spim::Spim::new(
        b.SPIM0,
        tft_spim_pins,
        spim::Frequency::M8,
        spim::MODE_3,
        122,
    );
    let tft_display_interface = SPIInterfaceNoCS::new(tft_spi, b.tft.dc);
    let mut display = ST7789::new(tft_display_interface, b.tft.reset, TFT::XSIZE, TFT::YSIZE);
    let mut delay = Delay::new(b.core_peripherals.SYST);
    display.init(&mut delay).unwrap();
    display.set_orientation(Orientation::Landscape).unwrap();
    display.clear(Rgb565::BLACK).unwrap();

    // allow setting the gain dynamically using the buttons
    let buttons = b.buttons;
    let mut decrease_gain = ButtonUpDown::new(buttons.button_a);
    let mut increase_gain = ButtonUpDown::new(buttons.button_b);
    // we need the high frequency oscillator enabled for the microphone
    let c = Clocks::new(b.CLOCK);
    c.enable_ext_hfosc();
    let mut mic = Microphone::new(b.PDM);
    mic.enable();
    let mut gain: i8 = 0;
    mic.set_gain(gain);
    const SAMPLECOUNT: u16 = 2048;
    const BUCKETCOUNT: usize = 120;
    // will drop some off at the end if they don't divide evenly, but going to ignore that
    const SAMPLESPERBUCKET: usize = SAMPLECOUNT as usize / BUCKETCOUNT;
    let pdmbuffers: [[u16; SAMPLECOUNT as usize]; 2] = [[0; SAMPLECOUNT as usize]; 2];
    let mut buckets: [u32; BUCKETCOUNT] = [0; BUCKETCOUNT];
    const BARWIDTH: u16 = TFT::XSIZE / BUCKETCOUNT as u16;
    assert_ne!(BARWIDTH, 0);
    let rect = |b, y, height, c| {
        Rectangle::new(
            Point::new((b as u32 * BARWIDTH as u32).try_into().unwrap(), y),
            Size::new(BARWIDTH.into(), height),
        )
        .into_styled(PrimitiveStyle::with_fill(c))
    };
    let mut buf_to_display: usize = 0;
    let mut buf_to_capture: usize = 1;
    mic.set_sample_buffer(&pdmbuffers[buf_to_capture]);
    mic.start_sampling();
    decrease_gain.update_state();
    increase_gain.update_state();
    loop {
        // compute bucketed average of the buffer to display
        for b in 0..BUCKETCOUNT {
            let start_at = b * SAMPLESPERBUCKET;
            let avg = mean(&pdmbuffers[buf_to_display][start_at..start_at + SAMPLESPERBUCKET]);
            let scaled_avg: u32 = ((avg / 65535.0) * TFT::YSIZE as f32) as u32;
            let prev_scaled_avg = buckets[b];
            if scaled_avg > prev_scaled_avg {
                // Extend the existing bar to match what we have now
                rect(
                    b,
                    prev_scaled_avg.try_into().unwrap(),
                    scaled_avg - prev_scaled_avg,
                    Rgb565::WHITE,
                )
                .draw(&mut display)
                .unwrap();
            } else {
                // Truncate the existing bar to match what we have now
                rect(
                    b,
                    scaled_avg.try_into().unwrap(),
                    prev_scaled_avg - scaled_avg,
                    Rgb565::BLACK,
                )
                .draw(&mut display)
                .unwrap();
            }
            buckets[b] = scaled_avg;
        }
        (buf_to_capture, buf_to_display) = if buf_to_capture == 0 { (1, 0) } else { (0, 1) };
        // Wait for the current buffer capture to complete, then start on the flipped buffer
        while !mic.sampling_started() {
            let mut gain_change: i8 = 0;
            if increase_gain.button_up() {
                gain_change += 1;
            }
            if decrease_gain.button_up() {
                gain_change -= 1;
            }
            if gain_change != 0 {
                gain += gain_change;
                gain = mic.set_gain(gain);
            }
        }
        mic.clear_sampling_started();
        mic.set_sample_buffer(&pdmbuffers[buf_to_capture]);
    }
}

fn mean(samples: &[u16]) -> f32 {
    let mut avg: f32 = 0.0;
    for s in samples {
        avg += *s as f32;
    }
    avg / (samples.len() as f32)
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
