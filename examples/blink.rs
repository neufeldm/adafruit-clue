#![no_main]
#![no_std]

use adafruit_clue::Board;
use cortex_m_rt;
use embedded_hal::blocking::delay::DelayMs;
use nrf52840_hal::Timer;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut b = Board::take().unwrap();

    let mut timer = Timer::new(b.TIMER4);

    loop {
        // Blink some LEDs
        b.leds.red.on();
        b.leds.white.off();
        timer.delay_ms(500 as u32);
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
