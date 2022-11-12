#![no_main]
#![no_std]

use core::convert::TryInto;
use adafruit_clue::{Board,TFT};
use cortex_m_rt;
use embedded_hal::blocking::delay::DelayMs;
use nrf52840_hal::clocks::Clocks;
use nrf52840_hal::{Timer,spim,Delay,gpio};

use display_interface_spi::SPIInterfaceNoCS;
use st7789::{Orientation, ST7789};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::*;
use embedded_graphics::prelude::*;

#[cortex_m_rt::entry]
unsafe fn main() -> ! {
    let mut b = Board::take().unwrap();
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
    // rig pins into proper mode
    b.pins.pdm_clock.into_push_pull_output(gpio::Level::Low);
    b.pins.pdm_data.into_floating_input();

    // we need the high frequency oscillator enabled
    Clocks::new(b.CLOCK).enable_ext_hfosc();

    // configure the PDM to use the correct pins
    b.PDM.psel.clk.write(|w| {
        w.port().bit(Board::PDM_CLOCK_PORT);
        // XXX unsafe
        w.pin().bits(Board::PDM_CLOCK_PIN);
        w.connect().clear_bit()
    });
    b.PDM.psel.din.write(|w| {
        w.port().bit(Board::PDM_DATA_PORT);
        // XXX unsafe
        w.pin().bits(Board::PDM_DATA_PIN);
        w.connect().clear_bit()
    });
    // enable the PDM
    b.PDM.enable.write(|w| { w.enable().set_bit() });
    // mono/rising edge
    b.PDM.mode.write(|w| {
        w.operation().set_bit();
        w.edge().clear_bit()
    });
    // XXX leaving gain, etc. at default values for now...
    const SAMPLECOUNT: u16 = 2048;
    const BUCKETCOUNT: usize = 120;
    // will drop some off at the end if they don't divide evenly, but going to ignore that
    const SAMPLESPERBUCKET: usize = SAMPLECOUNT as usize /BUCKETCOUNT;
    let pdmbuffers: [[u16;SAMPLECOUNT as usize];2] = [[0;SAMPLECOUNT as usize];2];
    let mut buckets: [u32;BUCKETCOUNT] = [0;BUCKETCOUNT];
    const BARWIDTH: u16 = TFT::XSIZE/BUCKETCOUNT as u16;
    assert_ne!(BARWIDTH,0);
    let rect = | b , y, height, c | {
        Rectangle::new(Point::new((b as u32*BARWIDTH as u32).try_into().unwrap(),y), Size::new(BARWIDTH.into(),height)).into_styled(PrimitiveStyle::with_fill(c))
    };
    b.PDM.sample.maxcnt.write(|w|{ w.bits(SAMPLECOUNT.into()) });
    let mut buf_to_display: usize = 0;
    let mut buf_to_capture: usize = 1;
    b.PDM.sample.ptr.write(|w| { w.bits(pdmbuffers[buf_to_capture].as_ptr() as u32)} );
    b.PDM.tasks_start.write(|w| { w.tasks_start().set_bit() });
    loop {
        // compute bucketed average of the buffer to display
        for b in 0..BUCKETCOUNT {
            let start_at = b*SAMPLESPERBUCKET;
            let avg = mean(&pdmbuffers[buf_to_display][start_at..start_at+SAMPLESPERBUCKET]);
            let scaled_avg: u32 = ((avg/65535.0) * TFT::YSIZE as f32) as u32;
            let prev_scaled_avg = buckets[b];
            if scaled_avg > prev_scaled_avg {
                // Extend the existing bar to match what we have now
                rect(b,prev_scaled_avg.try_into().unwrap(),scaled_avg - prev_scaled_avg,Rgb565::WHITE).draw(&mut display).unwrap();
            } else {
                // Truncate the existing bar to match what we have now
                rect(b,scaled_avg.try_into().unwrap(),prev_scaled_avg - scaled_avg,Rgb565::BLACK).draw(&mut display).unwrap();
            }
            buckets[b] = scaled_avg;
        }
        (buf_to_capture,buf_to_display) = if buf_to_capture == 0 { (1,0) } else { (0,1) };
        // Wait for the current buffer capture to complete, then start on the flipped buffer
        let sampling_started = &b.PDM.events_started;
        while !sampling_started.read().events_started().bit_is_set() {}
        sampling_started.write(|w| { w.events_started().clear_bit() });
        b.PDM.sample.ptr.write(|w| { w.bits(pdmbuffers[buf_to_capture].as_ptr() as u32)} );
    }
}

fn mean(samples: &[u16]) -> f32 {
    let mut avg: f32 = 0.0;
    for s in samples {
        avg += *s as f32;
    }
    avg/(samples.len() as f32)
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
