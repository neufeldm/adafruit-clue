//! Board support crate for the Nordic nRF52840-DK
//!
//! UARTE, SPIM and TWI should be functional,
//! but might miss some features.
#![no_std]

pub use cortex_m;
pub use embedded_hal;
pub use nrf52840_hal as hal;

//use nrf52840_hal::twim;

/// Exports traits that are usually needed when using this crate
pub mod prelude {
    pub use nrf52840_hal::prelude::*;
}

// TODO: Maybe we want a debug module like in the DWM1001-Dev implementation.
// pub mod debug;

use nrf52840_hal::{
    gpio::{p0, p1, Disconnected, Input, Level, Output, Pin, PullUp, PushPull,Floating},
    pac::{self as pac,CorePeripherals, Peripherals},
    //spim,
    //spim::{self, Frequency, Spim, MODE_0},
    //uarte::{self, Baudrate as UartBaudrate, Parity as UartParity, Uarte},
};

use embedded_hal::digital::v2::{InputPin, OutputPin};

/// Provides access to all features of the nRF52840-DK board
#[allow(non_snake_case)]
pub struct Board {
    pub corePeripherals: CorePeripherals,
    //pub peripherals: Peripherals,
    /// nRF52 peripheral: TIMER0
    pub TIMER0: pac::TIMER0,

    /// nRF52 peripheral: TIMER1
    pub TIMER1: pac::TIMER1,

    /// nRF52 peripheral: TIMER2
    pub TIMER2: pac::TIMER2,

    /// nRF52 peripheral: TIMER3
    pub TIMER3: pac::TIMER3,

    /// nRF52 peripheral: TIMER4
    pub TIMER4: pac::TIMER4,

    /// The nRF52's pins which are not otherwise occupied on the nRF52840-DK
    pub pins: Pins,

    // XXX mneufeld clue is supposed to have 2MB of QSPI flash - figure this out
    /// The nRF52840-DK SPI which is wired to the SPI flash
    //pub flash: Spim<nrf52::SPIM2>,
    //pub flash_cs: Pin<Output<PushPull>>,

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
    /// This method will return an instance of `nRF52840DK` the first time it is
    /// called. It will return only `None` on subsequent calls.
    pub fn take() -> Option<Self> {
        Some(Self::new(CorePeripherals::take()?, Peripherals::take()?))
    }

    /// Steal the peripherals
    ///
    /// This method produces an instance of `nRF52840DK`, regardless of whether
    /// another instance was create previously.
    ///
    /// # Safety
    ///
    /// This method can be used to create multiple instances of `nRF52840DK`. Those
    /// instances can interfere with each other, causing all kinds of unexpected
    /// behavior and circumventing safety guarantees in many ways.
    ///
    /// Always use `nRF52840DK::take`, unless you really know what you're doing.
    pub unsafe fn steal() -> Self {
        Self::new(CorePeripherals::steal(), Peripherals::steal())
    }

    fn new(cp: CorePeripherals, p: Peripherals) -> Self {
        let pins0 = p0::Parts::new(p.P0);
        let pins1 = p1::Parts::new(p.P1);


        // XXX mneufeld clue is supposed to have 2MB of QSPI flash - figure out where
        // The nRF52840-DK has an 64MB SPI flash on board which can be interfaced through SPI or Quad SPI.
        // As for now, only the normal SPI mode is available, so we are using this for the interface.
        //let flash_spim = Spim::new(
        //    p.SPIM2,
        //    spim::Pins {
        //        sck: pins0.p0_19.into_push_pull_output(Level::Low).degrade(),
        //        mosi: Some(pins0.p0_20.into_push_pull_output(Level::Low).degrade()),
        //        miso: Some(pins0.p0_21.into_floating_input().degrade()),
        //    },
        //    Frequency::K500,
        //    MODE_0,
        //    0,
        //);
        //let flash_cs = pins0.p0_17.into_push_pull_output(Level::High).degrade();

        //let tft_spim = Spim::new(
        //    p.SPIM0,
        //    spim::Pins {
        //        sck: pins0.p0_14.into_push_pull_output(Level::Low).degrade(),
        //        mosi: Some(pins0.p0_15.into_push_pull_output(Level::Low).degrade()),
        //        miso: None,
        //    },
        //    Frequency::K500, // XXX see if this is right...
        //    MODE_0,
        //    0,
        //);
        Board {
            // XXX mneufeld figure this out for clue
            //flash: flash_spim,
            //flash_cs,
            corePeripherals: cp,
            //peripherals: p,

            TIMER0: p.TIMER0,
            TIMER1: p.TIMER1,
            TIMER2: p.TIMER2,
            TIMER3: p.TIMER3,
            TIMER4: p.TIMER4,

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
            // XXX CONFIG_NFCT_PINS_AS_GPIOS (done in NVRAM already?)
            // p0_09 is proximity light interrupt on clue
            // p0_10 is the white led
            //nfc: NFC {
            //    nfc_1: pins0.p0_09,
            //    nfc_2: pins0.p0_10,
            //},
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

    // I2C sensor bus
    //pub SCL: p0::P0_25<Disconnected>, // (GPIO D19 / SCL)
    //pub SDA: p0::P0_24<Disconnected>, // (GPIO D20 / SDA)

    //pub BUTTON_A: p1::P1_02<Disconnected>, // (GPIO D5 / Left button)
    //pub BUTTON_B: p1::P1_10<Disconnected>, // (GPIO D11 / Right Button)

    //pub RED_LED: p1::P1_01<Disconnected>, // (Red LED) circuitpython uses "L" for the name...
    //pub NEOPIXEL: p0::P0_16<Disconnected>, // (NeoPixel)

    //pub MICROPHONE_DATA: p0::P0_00<Disconnected>, // (PDM DAT)
    //pub MICROPHONE_CLOCK: p0::P0_01<Disconnected>, // (PDM CLOCK)

    //pub ACCELEROMETER_GYRO_INTERRUPT: p1::P1_06<Disconnected>, // (LSM6DS33 IRQ)
    //pub PROXIMITY_LIGHT_INTERRUPT: p0::P0_09<Disconnected>, // (APDS IRQ)
    //pub SPEAKER: p1::P1_00, // (Speaker/buzzer)


    // TFT display 
    //pub TFT_SCK: p0::P0_14,
    //pub TFT_MOSI: p0::P0_15,
    //pub TFT_CS: p0::P0_12,
    //pub TFT_DC: p0::P0_13,
    //pub TFT_RESET: p1::P1_03,
    //pub TFT_BACKLIGHT: p1::P1_05,
  
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
    /// Adafruit Clue: Red LED, nrf52: P1.01
    pub red: Led,

    /// Adafruit Clue: White LED, nRF52: P0.10
    pub white: Led,
}

/// An LED on the nRF52840-DK board
pub struct Led(Pin<Output<PushPull>>);

impl Led {
    fn new<Mode>(pin: Pin<Mode>) -> Self {
        Led(pin.into_push_pull_output(Level::High))
    }

    /// Release the inner Pin to be used directly
    pub fn release(self) -> Pin<Output<PushPull>> {
        self.0
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

/// The Buttons on the Adafruit Clue
pub struct Buttons {
    /// adafruit clue: Button a, nRF52: P1.02
    pub button_a: Button,
    /// adafruit clue: Button b, nRF52: P1.10
    pub button_b: Button,
}

/// Button on the Clue
pub struct Button(Pin<Input<PullUp>>);

impl Button {
    fn new<Mode>(pin: Pin<Mode>) -> Self {
        Button(pin.into_pullup_input())
    }

    /// Release the inner Pin to be used directly
    pub fn release(self) -> Pin<Input<PullUp>> {
        self.0
    }

    /// Button is pressed
    pub fn is_pressed(&self) -> bool {
        self.0.is_low().unwrap()
    }

    /// Button is released
    pub fn is_released(&self) -> bool {
        self.0.is_high().unwrap()
    }
}

pub struct TFT {
    pub sck: Pin<Output<PushPull>>,
    pub mosi: Pin<Output<PushPull>>,
    pub cs: Pin<Output<PushPull>>,
    pub dc: Pin<Output<PushPull>>,
    pub reset: Pin<Output<PushPull>>,
    pub backlight: Pin<Output<PushPull>>,
    //pub spim: Option<spim::Spim>,
}

impl TFT {
    pub const XSIZE: u16 = 240;
    pub const YSIZE: u16 = 240;
    // XXX how to pass in the spim?
    //pub fn setupSPIM<SPIMDEV>(self,spimdev: SPIMDEV) {
    //    let tft_pins = spim::Pins {
    //        sck: self.tft_sck,
    //        miso: None,
    //        mosi: Some(self.tft_mosi),
    //    };
    //    self.spim = spim::Spim::new(spimdev,tft_pins,spim::Frequency::M8,spim::MODE_3,122);
    //}
}

pub struct Sensors {
    pub sda: Pin<Input<Floating>>,
    pub scl: Pin<Input<Floating>>,
    //pub twim: Option<twim::Twim>,
}

impl Sensors {
    pub const GYROACCEL: u8 = 0x6A;
    pub const MAGNETOMETER: u8 = 0x1c;
    pub const GESTURE: u8 = 0x39;
    pub const HUMIDITY: u8 = 0x44;
    pub const TEMPPRESSUE: u8 = 0x77;
    //pub fn setupTWIM<TWIMDEV>(self, twimdev: TWIMDEV) {
        //let sensor_i2c_pins = twim::Pins {
            //scl: self.scl,
            //sda: self.sda,
        //};
        //self.twim = twim::Twim::new(twimdev,sensor_i2c_pins,twim::Frequency::K400);
    //}
}
// The NFC pins on the nRF52840-DK board
// XXX configure these as GPIO for the clue
//pub struct NFC {
//    /// nRF52840-DK: NFC1, nRF52: P0.09
//    pub nfc_1: p0::P0_09<Disconnected>,
//
//    /// nRF52840-DK: NFC2, nRF52: P0.10
//    pub nfc_2: p0::P0_10<Disconnected>,
//}
