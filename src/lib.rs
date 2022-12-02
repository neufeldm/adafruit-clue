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
    gpio::{p0, p1, Disconnected, Floating, Input, Level, Output, Pin, PullUp, PushPull},
    pac::{self as pac, CorePeripherals, Peripherals},
    //spim,
    //spim::{self, Frequency, Spim, MODE_0},
    //uarte::{self, Baudrate as UartBaudrate, Parity as UartParity, Uarte},
};

use embedded_hal::digital::v2::{InputPin, OutputPin};

/// Provides access to features of the CLUE board
#[allow(non_snake_case)]
pub struct Board {
    pub core_peripherals: CorePeripherals,

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

    pub UARTE0: pac::UARTE0,
    pub UARTE1: pac::UARTE1,

    pub USBD: pac::USBD,
    pub CLOCK: pac::CLOCK,
    pub PDM: pac::PDM,

    /// The nRF52's pins which are not otherwise occupied on the nRF52840-DK
    pub pins: Pins,

    // XXX mneufeld clue is supposed to have 2MB of QSPI flash - figure this out
    /// The LEDs on the Clue
    pub leds: Leds,

    /// The buttons on the Clue
    pub buttons: Buttons,

    /// The Clue TFT pins
    pub tft: TFT,

    /// Clue I2C pins for sensors
    pub sensors_i2c: I2C,
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

    // USB CDC identifiers
    pub const USB_PRODUCT: &'static str = "Adafruit CLUE";
    pub const USB_MANUFACTURER: &'static str = "Adafruit";
    pub const USB_VID: u16 = 0x239A;
    pub const USB_PID: u16 = 0x8072;

    // Sensor I2C IDs
    pub const I2C_GYROACCEL: u8 = 0x6A;
    pub const I2C_MAGNETOMETER: u8 = 0x1c;
    pub const I2C_GESTURE: u8 = 0x39;
    pub const I2C_HUMIDITY: u8 = 0x44;
    pub const I2C_TEMPPRESSURE: u8 = 0x77;

    // PDM port/pin IDs
    pub const PDM_DATA_PORT: bool = false;
    pub const PDM_DATA_PIN: u8 = 0x00;
    pub const PDM_CLOCK_PORT: bool = false;
    pub const PDM_CLOCK_PIN: u8 = 0x01;

    fn new(cp: CorePeripherals, p: Peripherals) -> Self {
        let pins0 = p0::Parts::new(p.P0);
        let pins1 = p1::Parts::new(p.P1);

        Board {
            core_peripherals: cp,

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

            UARTE0: p.UARTE0,
            UARTE1: p.UARTE1,

            USBD: p.USBD,
            CLOCK: p.CLOCK,
            PDM: p.PDM,

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
                neopixel: pins0.p0_16,
                pdm_data: pins0.p0_00.into_floating_input(),
                pdm_clock: pins0.p0_01.into_push_pull_output(Level::Low),
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

            sensors_i2c: I2C {
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

    // XXX resolve these
    //pub SCK: p0::P0_08<Disconnected>, // (GPIO D13 / SCK)
    //pub MISO: p0::P0_06<Disconnected>, // (GPIO D14 / MISO)
    //pub MOSI: p0::P0_26<Disconnected>, // (GPIO D15 / MOSI)
    pub neopixel: p0::P0_16<Disconnected>, // (NeoPixel)

    pub pdm_data: p0::P0_00<Input<Floating>>, // (PDM DAT)
    pub pdm_clock: p0::P0_01<Output<PushPull>>, // (PDM CLOCK)

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
        Led(pin.into_push_pull_output(Level::Low))
    }

    /// Turn the LED on
    pub fn on(&mut self) {
        self.0.set_high().unwrap()
    }

    /// Turn the LED off
    pub fn off(&mut self) {
        self.0.set_low().unwrap()
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

#[derive(Clone, Copy, PartialEq)]
pub enum ButtonState {
    UNKNOWN,
    PRESSED,
    RELEASED,
}

pub struct ButtonUpDown {
    button: Button,
    prev_state: ButtonState,
}

impl ButtonUpDown {
    pub fn new(button: Button) -> Self {
        ButtonUpDown {
            button: button,
            prev_state: ButtonState::UNKNOWN,
        }
    }
    pub fn pressed(&self) -> bool {
        self.button.pressed()
    }
    pub fn update_state(&mut self) -> (ButtonState, ButtonState) {
        let prev_state_orig = self.prev_state;
        let cur_state = match self.pressed() {
            false => ButtonState::RELEASED,
            true => ButtonState::PRESSED,
        };
        self.prev_state = cur_state;
        (prev_state_orig, cur_state)
    }
    pub fn button_down(&mut self) -> bool {
        self.update_state() == (ButtonState::RELEASED, ButtonState::PRESSED)
    }
    pub fn button_up(&mut self) -> bool {
        self.update_state() == (ButtonState::PRESSED, ButtonState::RELEASED)
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
    pub fn backlight_on(&mut self) {
        self.backlight.set_high().unwrap();
    }
    pub fn backlight_off(&mut self) {
        self.backlight.set_low().unwrap();
    }
}

/// I2C pins for Clue sensors
pub struct I2C {
    pub sda: Pin<Input<Floating>>,
    pub scl: Pin<Input<Floating>>,
}

pub struct Microphone {
    pdm: pac::PDM,
}

impl Microphone {
    pub fn new(pdm: pac::PDM) -> Self {
        Microphone { pdm: pdm }
    }
    pub fn enable(&mut self) {
        // configure the PDM to use the correct pins
        self.pdm.psel.clk.write(|w| {
            w.port().bit(Board::PDM_CLOCK_PORT);
            unsafe {
                w.pin().bits(Board::PDM_CLOCK_PIN);
            }
            w.connect().clear_bit()
        });
        self.pdm.psel.din.write(|w| unsafe {
            w.port().bit(Board::PDM_DATA_PORT);
            w.pin().bits(Board::PDM_DATA_PIN);
            w.connect().clear_bit()
        });
        // enable the PDM
        self.pdm.enable.write(|w| w.enable().set_bit());
        // mono/rising edge
        self.pdm.mode.write(|w| {
            w.operation().set_bit();
            w.edge().clear_bit()
        });
    }
    pub fn enable_interrupt() {}
    pub fn disable_interrupt() {}
    pub fn set_sample_buffer(&self, buf: &[u16]) {
        unsafe {
            self.pdm.sample.ptr.write(|w| w.bits(buf.as_ptr() as u32));
            self.pdm.sample.maxcnt.write(|w| w.bits(buf.len() as u32));
        }
    }
    pub const MIN_GAIN_HALFDB: i8 = -40;
    pub const MAX_GAIN_HALFDB: i8 = 40;
    pub fn set_gain(&self, half_db_gain: i8) -> i8 {
        let mut g = half_db_gain;
        unsafe {
            if half_db_gain < Microphone::MIN_GAIN_HALFDB {
                g = Microphone::MIN_GAIN_HALFDB;
            } else if half_db_gain > Microphone::MIN_GAIN_HALFDB {
                g = Microphone::MAX_GAIN_HALFDB;
            }
            // The gain is offset by the minimum gain to 0, so adjust
            // the signed value here to match what the register wants.
            g -= Microphone::MIN_GAIN_HALFDB;
            self.pdm.gainl.write(|w| w.gainl().bits(g as u8));
            self.pdm.gainr.write(|w| w.gainr().bits(g as u8));
        }
        g
    }

    pub fn start_sampling(&self) {
        self.pdm.tasks_start.write(|w| w.tasks_start().set_bit());
    }

    pub fn sampling_started(&self) -> bool {
        self.pdm.events_started.read().events_started().bit_is_set()
    }

    pub fn clear_sampling_started(&self) {
        self.pdm
            .events_started
            .write(|w| w.events_started().clear_bit());
    }

    pub fn stop_sampling(&self) {
        self.pdm.tasks_stop.write(|w| w.tasks_stop().set_bit());
    }
    pub fn sampling_stopped(&self) -> bool {
        self.pdm.events_stopped.read().events_stopped().bit_is_set()
    }
    pub fn clear_sampling_stopped(&self) {
        self.pdm
            .events_stopped
            .write(|w| w.events_stopped().clear_bit());
    }

    pub fn sampling_ended(&self) -> bool {
        self.pdm.events_end.read().events_end().bit_is_set()
    }

    pub const IRQ_SAMPLING_STARTED: u32 = 0b001;
    pub const IRQ_SAMPLING_STOPPED: u32 = 0b010;
    pub const IRQ_SAMPLING_ENDED: u32 = 0b100;
    pub const IRQ_SAMPLING_ALL: u32 = 0b111;
    pub const IRQ_SAMPLING_NONE: u32 = 0b000;
    pub fn enable_interrupts(&self) {
        unsafe {
            self.pdm
                .inten
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_ALL))
        }
    }

    pub fn disable_interrupts(&self) {
        unsafe {
            self.pdm
                .inten
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_NONE))
        }
    }

    pub fn enable_started_interrupt(&self) {
        unsafe {
            self.pdm
                .intenset
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_STARTED))
        }
    }
    pub fn disable_started_interrupt(&self) {
        unsafe {
            self.pdm
                .intenclr
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_STARTED))
        }
    }

    pub fn enable_stopped_interrupt(&self) {
        unsafe {
            self.pdm
                .intenset
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_STOPPED))
        }
    }
    pub fn disable_stopped_interrupt(&self) {
        unsafe {
            self.pdm
                .intenclr
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_STOPPED))
        }
    }

    pub fn enable_ended_interrupt(&self) {
        unsafe {
            self.pdm
                .intenset
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_ENDED))
        }
    }
    pub fn disable_ended_interrupt(&self) {
        unsafe {
            self.pdm
                .intenclr
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_ENDED))
        }
    }
}
