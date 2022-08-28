//! Board support for Adafruit CLUE:
//! https://www.adafruit.com/product/4500
//!
//! Heavily based on existing board support crates:
//! https://github.com/nrf-rs/nrf52840-dk
//! https://github.com/nrf-rs/adafruit-nrf52-bluefruit-le

#![no_std]

pub use cortex_m;
pub use embedded_hal;
pub use nrf52840_hal as hal;

/// Exports traits that are usually needed when using this crate
pub mod prelude {
    pub use nrf52840_hal::prelude::*;
}

use nrf52840_hal::{
    gpio::{p0, p1, Disconnected, Input, Level, Output, Pin, PullUp, PushPull,Floating},
    pac::{self as pac,CorePeripherals, Peripherals},
    //spim,
    //spim::{self, Frequency, Spim, MODE_0},
    //uarte::{self, Baudrate as UartBaudrate, Parity as UartParity, Uarte},
};

use embedded_hal::digital::v2::{InputPin, OutputPin};

/// Provides access to features of the CLUE board
#[allow(non_snake_case)]
pub struct Board {
    pub corePeripherals: CorePeripherals,

    pub TIMER0: pac::TIMER0,
    pub TIMER1: pac::TIMER1,
    pub TIMER2: pac::TIMER2,
    pub TIMER3: pac::TIMER3,
    pub TIMER4: pac::TIMER4,

    pub TWIM0: pac::TWIM0,
    pub TWIM1: pac::TWIM1,

    pub SPIM0: pac::SPIM0,
    pub SPIM1: pac::SPIM1,
    pub SPIM2: pac::SPIM2,
    pub SPIM3: pac::SPIM3,

    /// The nRF52's pins which are not otherwise occupied on the nRF52840-DK
    pub pins: Pins,

    // XXX mneufeld clue is supposed to have 2MB of QSPI flash - figure this out

    /// The LEDs on the Clue
    pub leds: Leds,

    /// The buttons on the Clue
    pub buttons: Buttons,

    /// The Clue TFT pins
    pub tft: TFT,

    pub sensors: Sensors,
}

impl Board {
    /// Take the peripherals safely
    ///
    /// This method will return an instance of `adafruit-clue` the first time it is
    /// called. It will return only `None` on subsequent calls.
    pub fn take() -> Option<Self> {
        Some(Self::new(CorePeripherals::take()?, Peripherals::take()?))
    }

    /// Steal the peripherals
    ///
    /// This method produces an instance of `adafruit-clue`, regardless of whether
    /// another instance was create previously.
    ///
    /// # Safety
    ///
    /// This method can be used to create multiple instances of `adafruit-clue`. Those
    /// instances can interfere with each other, causing all kinds of unexpected
    /// behavior and circumventing safety guarantees in many ways.
    ///
    /// Always use `adafruit-clue::take`, unless you really know what you're doing.
    pub unsafe fn steal() -> Self {
        Self::new(CorePeripherals::steal(), Peripherals::steal())
    }

    fn new(cp: CorePeripherals, p: Peripherals) -> Self {
        let pins0 = p0::Parts::new(p.P0);
        let pins1 = p1::Parts::new(p.P1);

        Board {
            corePeripherals: cp,

            TIMER0: p.TIMER0,
            TIMER1: p.TIMER1,
            TIMER2: p.TIMER2,
            TIMER3: p.TIMER3,
            TIMER4: p.TIMER4,

            TWIM0: p.TWIM0,
            TWIM1: p.TWIM1,

            SPIM0: p.SPIM0,
            SPIM1: p.SPIM1,
            SPIM2: p.SPIM2,
            SPIM3: p.SPIM3,

            pins: Pins {
                a0: pins0.p0_31,
                a1: pins0.p0_29,
                a2: pins0.p0_04,
                a3: pins0.p0_05,
                a4: pins0.p0_03,
                a5: pins0.p0_28,
                a6: pins0.p0_02,
                a7: pins0.p0_30,
                d6: pins1.p1_09,
                d7: pins0.p0_07,
                d8: pins1.p1_07,
                d9: pins0.p0_27,
            },
            tft: TFT {
                sck: pins0.p0_14.into_push_pull_output(Level::Low).degrade(),
                mosi: pins0.p0_15.into_push_pull_output(Level::Low).degrade(),
                cs: pins0.p0_12.into_push_pull_output(Level::Low).degrade(),
                dc: pins0.p0_13.into_push_pull_output(Level::Low).degrade(),
                reset: pins1.p1_03.into_push_pull_output(Level::Low).degrade(),
                backlight: pins1.p1_05.into_push_pull_output(Level::Low).degrade(),
            },
            leds: Leds {
                red: Led::new(pins1.p1_01.degrade()),
                white: Led::new(pins0.p0_10.degrade()),
            },

            buttons: Buttons {
                button_a: Button::new(pins1.p1_02.degrade()),
                button_b: Button::new(pins1.p1_10.degrade()),
            },

            sensors: Sensors {
                sda: pins0.p0_24.into_floating_input().degrade(),
                scl: pins0.p0_25.into_floating_input().degrade(),
            },
        }
    }
}

/// The nRF52 pins that are available on the Adafruit Clue
pub struct Pins {
    // XXX similar to circuitpython names here...
    pub a0: p0::P0_31<Disconnected>, // (GPIO D12 / AIN0)
    pub a1: p0::P0_29<Disconnected>, // (GPIO D16 / AIN1)
    pub a2: p0::P0_04<Disconnected>, // (GPIO D0 / AIN2 / UART RX)
    pub a3: p0::P0_05<Disconnected>, // (GPIO D1 / AIN3 / UART TX)
    pub a4: p0::P0_03<Disconnected>, // (GPIO D2 / AIN4)
    pub a5: p0::P0_28<Disconnected>, // (GPIO D3 / AIN5)
    pub a6: p0::P0_02<Disconnected>, // (GPIO D4 / AIN6)
    pub a7: p0::P0_30<Disconnected>, // (GPIO D10 / AIN7)

    pub d6: p1::P1_09<Disconnected>, // (GPIO D6)
    pub d7: p0::P0_07<Disconnected>, // (GPIO D7)
    pub d8: p1::P1_07<Disconnected>, // (GPIO D8)
    pub d9: p0::P0_27<Disconnected>, // (GPIO D9)

    // XXX primary UART?
    //pub SCK: p0::P0_08<Disconnected>, // (GPIO D13 / SCK)
    //pub MISO: p0::P0_06<Disconnected>, // (GPIO D14 / MISO)
    //pub MOSI: p0::P0_26<Disconnected>, // (GPIO D15 / MOSI)

    //pub NEOPIXEL: p0::P0_16<Disconnected>, // (NeoPixel)

    //pub MICROPHONE_DATA: p0::P0_00<Disconnected>, // (PDM DAT)
    //pub MICROPHONE_CLOCK: p0::P0_01<Disconnected>, // (PDM CLOCK)

    //pub ACCELEROMETER_GYRO_INTERRUPT: p1::P1_06<Disconnected>, // (LSM6DS33 IRQ)
    //pub PROXIMITY_LIGHT_INTERRUPT: p0::P0_09<Disconnected>, // (APDS IRQ)
    //pub SPEAKER: p1::P1_00, // (Speaker/buzzer)
  
    // QSPI pins (not exposed via any header / test point)
    //pub QSPI_CLK: p0::P0_19, // (QSPI CLK)
    //pub QSPI_CS: p0::P0_20, // (QSPI CS)
    //pub QSPI_DATA0: p0::P0_17, // (QSPI Data 0)
    //pub QSPI_DATA1: p0::P0_22, // (QSPI Data 1)
    //pub QSPI_DATA2: p0::P0_23, // (QSPI Data 2)
    //pub QSPI_DATA3: p0::P0_21, // (QSPI Data 3)
}

/// The LEDs on the Adafruit Clue
pub struct Leds {
    /// Red LED on back of Clue (P1_01 - circuitpython uses "L" for the name)
    pub red: Led,
    /// Twin white LEDs on front of Clue (P0_10)
    pub white: Led,
}

/// An LED control pin
pub struct Led(Pin<Output<PushPull>>);

impl Led {
    fn new<Mode>(pin: Pin<Mode>) -> Self {
        Led(pin.into_push_pull_output(Level::High))
    }

    /// Turn the LED on
    pub fn on(&mut self) {
        self.0.set_low().unwrap()
    }

    /// Turn the LED off
    pub fn off(&mut self) {
        self.0.set_high().unwrap()
    }
}

/// The buttons on the Adafruit Clue
pub struct Buttons {
    /// Button A (P1_02 / GPIO D5 / Left button)
    pub button_a: Button,
    /// Button B: (P1_10 / GPIO D11 / Right Button)
    pub button_b: Button,
}

/// Button on the Clue
pub struct Button(Pin<Input<PullUp>>);

impl Button {
    fn new<Mode>(pin: Pin<Mode>) -> Self {
        Button(pin.into_pullup_input())
    }

    /// Button is pressed
    pub fn pressed(&self) -> bool {
        self.0.is_low().unwrap()
    }
}

/// Control pins for the Clue TFT
pub struct TFT {
    pub sck: Pin<Output<PushPull>>,
    pub mosi: Pin<Output<PushPull>>,
    pub cs: Pin<Output<PushPull>>,
    pub dc: Pin<Output<PushPull>>,
    pub reset: Pin<Output<PushPull>>,
    pub backlight: Pin<Output<PushPull>>,
}

impl TFT {
    pub const XSIZE: u16 = 240;
    pub const YSIZE: u16 = 240;
    pub fn backlight_on(mut self) {
        self.backlight.set_high().unwrap();
    }
    pub fn backlight_off(mut self) {
        self.backlight.set_low().unwrap();
    }
}

/// I2C pins for Clue sensors
pub struct Sensors {
    pub sda: Pin<Input<Floating>>,
    pub scl: Pin<Input<Floating>>,
}

impl Sensors {
    pub const GYROACCEL: u8 = 0x6A;
    pub const MAGNETOMETER: u8 = 0x1c;
    pub const GESTURE: u8 = 0x39;
    pub const HUMIDITY: u8 = 0x44;
    pub const TEMPPRESSUE: u8 = 0x77;
}
