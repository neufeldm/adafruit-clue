#![no_main]
#![no_std]

use adafruit_clue::Board;
use cortex_m_rt;
use embedded_hal::blocking::delay::DelayMs;
use nrf52840_hal::{pwm, Timer, gpio};


const NEOPIXEL_0: u16 = 0x8005;
const NEOPIXEL_1: u16 = 0x800d;
const NEOPIXEL_LATCH: u16 = 0x8000;
const RGB_BITS: usize = 24;
const LATCH_CYCLES: usize = 40;
const LATCH_START: usize = 24;
const NEOPIXEL_SEQ_SIZE: usize = RGB_BITS + LATCH_CYCLES;
static mut NEOPIXEL_SEQ: [u16; NEOPIXEL_SEQ_SIZE] = [NEOPIXEL_LATCH; NEOPIXEL_SEQ_SIZE];

fn pwm_neo_init<P: nrf52840_hal::pwm::Instance>(rawpwm: P,pin: gpio::Pin<gpio::Output<gpio::PushPull>>) -> pwm::Pwm<P> {
    let pwmdev = pwm::Pwm::new(rawpwm);
    pwmdev.set_output_pin(
        pwm::Channel::C0,
        pin,
    );
    pwmdev.enable();
    pwmdev.set_counter_mode(pwm::CounterMode::Up)
          .set_prescaler(pwm::Prescaler::Div1) // 16MHz clock
          .set_max_duty(20) // 20 16Mhz clicks -> 800kHz
          .set_load_mode(pwm::LoadMode::Common)
          .set_seq_refresh(pwm::Seq::Seq0,0)
          .set_seq_end_delay(pwm::Seq::Seq0,0)
          .one_shot();
    pwmdev
}

fn pwm_neo_set<P: nrf52840_hal::pwm::Instance +core::fmt::Debug>(neopwm: pwm::Pwm<P>,red: u8,green: u8,blue: u8) {
    unsafe {
        NEOPIXEL_SEQ[0..LATCH_START].copy_from_slice(&mut pwm_neo_rgb(red,green,blue));
        neopwm.load(Some(&NEOPIXEL_SEQ),None::<&[u16; NEOPIXEL_SEQ_SIZE]>,true).unwrap();
    }
}

fn pwm_neo_bit(val: u8, bit: usize) -> u16 {
    if val & (0x1 << (7 - bit)) == 0 {
        NEOPIXEL_0
    } else {
        NEOPIXEL_1
    }
}

fn pwm_neo_rgb(red: u8, green: u8, blue: u8) -> [u16; 24] {
    [
        pwm_neo_bit(green,0),
        pwm_neo_bit(green,1),
        pwm_neo_bit(green,2),
        pwm_neo_bit(green,3),
        pwm_neo_bit(green,4),
        pwm_neo_bit(green,5),
        pwm_neo_bit(green,6),
        pwm_neo_bit(green,7),

        pwm_neo_bit(red,0),
        pwm_neo_bit(red,1),
        pwm_neo_bit(red,2),
        pwm_neo_bit(red,3),
        pwm_neo_bit(red,4),
        pwm_neo_bit(red,5),
        pwm_neo_bit(red,6),
        pwm_neo_bit(red,7),

        pwm_neo_bit(blue,0),
        pwm_neo_bit(blue,1),
        pwm_neo_bit(blue,2),
        pwm_neo_bit(blue,3),
        pwm_neo_bit(blue,4),
        pwm_neo_bit(blue,5),
        pwm_neo_bit(blue,6),
        pwm_neo_bit(blue,7),
    ]   
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut b = Board::take().unwrap();
    let neopwm = pwm_neo_init(b.PWM0,b.pins.neopixel.into_push_pull_output(gpio::Level::Low).degrade());
    pwm_neo_set(neopwm,0,0,255);

    let mut timer = Timer::new(b.TIMER4).into_periodic();
    loop {
        // Blink the red LED
        b.leds.red.on();
        timer.delay_ms(500 as u32);
        b.leds.red.off();
        timer.delay_ms(100 as u32);
    }
}

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
