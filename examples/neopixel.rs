#![no_main]
#![no_std]

use adafruit_clue::{Board, PWMNeoPixel, PWM_NEOPIXEL_SEQ_SIZE};
use cortex_m_rt;
use embedded_hal::blocking::delay::DelayMs;
use nrf52840_hal::Timer;

// The PWM devices requires a static buffer for DMA, and since
// we want to write different RGB values at runtime it needs to
// be mutable as well.
static mut PWM_NEOPIXEL_BUF: [u16; PWM_NEOPIXEL_SEQ_SIZE] = [0; PWM_NEOPIXEL_SEQ_SIZE];

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut b = Board::take().unwrap();
    // PWM DMA requires static lifetime and we need it mutable as well, so
    // our neopixel constructor is inherently unsafe.
    let mut neopixel =
        unsafe { PWMNeoPixel::new(b.PWM0, b.pins.neopixel.degrade(), &mut PWM_NEOPIXEL_BUF) };
    let mut timer = Timer::new(b.TIMER4).into_periodic();
    loop {
        // red
        neopixel.set_color(255, 0, 0);
        b.leds.red.on();
        timer.delay_ms(500 as u32);

        // green
        neopixel.set_color(0, 255, 0);
        b.leds.red.off();
        timer.delay_ms(500 as u32);

        // blue
        neopixel.set_color(0, 0, 255);
        b.leds.red.on();
        timer.delay_ms(500 as u32);

        // white
        neopixel.set_color(255, 255, 255);
        b.leds.red.off();
        timer.delay_ms(500 as u32);
    }
}

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
