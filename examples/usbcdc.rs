#![no_main]
#![no_std]

use adafruit_clue::Board;
use cortex_m_rt;
use embedded_hal::blocking::delay::DelayMs;
use nrf52840_hal::clocks::Clocks;
use nrf52840_hal::usbd::{UsbPeripheral, Usbd};
use nrf52840_hal::Timer;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[cortex_m_rt::entry]
fn main() -> ! {
    let b = Board::take().unwrap();
    let clocks = Clocks::new(b.CLOCK);
    let clocks = clocks.enable_ext_hfosc();

    let usb_bus = Usbd::new(UsbPeripheral::new(b.USBD, &clocks));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(Board::USB_VID, Board::USB_PID))
        .manufacturer(Board::USB_MANUFACTURER)
        .product(Board::USB_PRODUCT)
        .serial_number("CLUE")
        .device_class(USB_CLASS_CDC)
        .max_packet_size_0(64) // (makes control transfers 8x faster)
        .build();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back characters that came in
                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }
}

#[panic_handler] // panicking behavior
unsafe fn panic(_pinfo: &core::panic::PanicInfo) -> ! {
    let mut b: Board = Board::steal();
    let mut timer = Timer::new(b.TIMER3);
    loop {
        b.leds.red.on();
        timer.delay_ms(500 as u32);
        b.leds.red.off();
        timer.delay_ms(100 as u32);
        //cortex_m::asm::bkpt();
    }
}
