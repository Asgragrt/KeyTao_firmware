#![no_std]

use usbd_human_interface_device::page::Keyboard;
use embedded_hal::digital::v2::InputPin;
use heapless::Vec;
use libm::{powf, expf};

//Keys used
pub const KEY_COUNT: usize = 8;

//Struct for easier gpio -> keypress interface
#[derive(Copy, Clone)]
pub struct PinKeys<'a> {
    _pin_port: &'a dyn InputPin<Error = core::convert::Infallible>,
    _keys: &'a Vec<Keyboard, 4>, //Max 4 keys
}

impl<'a> PinKeys<'a> {
    pub fn new(pin_port: &'a dyn InputPin<Error = core::convert::Infallible>,
    keys: &'a Vec<Keyboard, 4>) -> Self {
        Self {
            _pin_port: pin_port,
            _keys: keys
        }
    }
    pub fn get_pin(&self) -> &'a dyn InputPin<Error = core::convert::Infallible> {
        self._pin_port
    }

    pub fn get_keys(&self) -> &'a Vec<Keyboard, 4> {
        self._keys
    }
}

#[macro_export]
macro_rules! def_key {
    ($pin_port: expr, $keys: expr) => {
        PinKeys {
            _pin_port: &($pin_port.into_pull_up_input()), 
            _keys: &Vec::from_slice(&$keys).unwrap()
        }        
    };
}

pub fn get_pin_keys<'a>(pin_ports: [&'a dyn InputPin<Error = core::convert::Infallible>; KEY_COUNT],
keys: [&'a Vec<Keyboard, 4>; KEY_COUNT]) -> [PinKeys<'a>; KEY_COUNT] {
    let mut arr = [PinKeys::new(pin_ports[0], keys[0]); KEY_COUNT];
    for i in 1..KEY_COUNT{
        arr[i] = PinKeys::new(pin_ports[i], keys[i]);
    }
    arr
}

#[macro_export]
macro_rules! pin_keys {
    ($($pin_port:expr, $keys:expr);*) => {
        (
        [$(&$pin_port.into_pull_up_input(),)*],
        [$(&Vec::from_slice(&$keys).unwrap(),)*]
        )      
    };
}

/*********LED CONTROL*/

pub const LOW: u16 = 0;
pub const HIGH: u16 = 55_000;

pub const LED_COUNT: i8 = 7;
pub const MODE_COUNT: u8 = 4;

#[macro_export]
macro_rules! def_pwm {
    ($pwm_ref:expr, [$($pwm_slice_ident:ident, $pwm_slice:ident, 
        [$($pin:expr, $pwm_channel:ident, $pwm_channel_ident:ident);*]);*]) => {
        $(
            let $pwm_slice_ident = &mut $pwm_ref.$pwm_slice;
            $pwm_slice_ident.set_ph_correct();
            $pwm_slice_ident.enable();
            
            $(
                let $pwm_channel_ident = &mut $pwm_slice_ident.$pwm_channel;
                $pwm_channel_ident.output_to($pin);
            )*
        )*
    };
}

#[macro_export]
macro_rules! min {
    ($x:expr, $y:expr) => {
        if $x as f32 >= $y as f32 {
            $y as f32
        } else{
            $x as f32
        }        
    };
}

#[macro_export]
macro_rules! pwm_mode {
    ($pin_modes:expr, [$($pwm_channel_ident:expr),*]) => {
        match $pin_modes.get_mode() {
            0 => {
                $pin_modes.breathing();
                $(
                    $pwm_channel_ident.set_duty($pin_modes.i);
                )*
            },
            1 => {
                $pin_modes.harsh_blink();
                $(
                    $pwm_channel_ident.set_duty($pin_modes.i);
                )*
            },
            2 => {
                $pin_modes.harsh_off();
                $(
                    $pwm_channel_ident.set_duty($pin_modes.i);
                )*
            },
            3 => {
                let mut led_count: i8 = 0;
                let mut dif: i8 = 0;
                let mut val: u16 = 0;
                let mut new_val: u16 = 0;
                
                $(
                    dif = ($pin_modes.counter - led_count).abs();
                    val = (HIGH as f32 * gaussian(min!((LED_COUNT - dif).abs(), dif) ) ) as u16;
                    if $pin_modes.initial {  
                        $pwm_channel_ident.set_duty(val);
                    }
                    else {
                        new_val = linear($pwm_channel_ident.get_duty(), val, 
                        $pin_modes._trail_speed - $pin_modes.get_timer());
                        
                        $pwm_channel_ident.set_duty(new_val);
                    }                    
                    
                    led_count += 1;
                )*
                $pin_modes.increase_timer();
            },

            _ => (),
        }
    };
}

pub fn gaussian(x: f32) -> f32 {
    expf(-powf((x as f32 / LED_COUNT as f32) - 0.5, 2.0) / (2.0*powf(0.1, 2.0)) )
}

pub fn linear(actual: u16, target: u16, dif: u16) -> u16 {
    let m: f32 = (target as f32 - actual as f32)  / (dif as f32);
    (m + (actual as f32)) as u16
}

pub struct PinModes {
    _mode: u8,
    pub i: u16,
    pub increasing: bool,
    pub counter: i8,
    _time_counter: u16,
    pub _trail_speed: u16,
    pub initial: bool,
}

impl PinModes {
    pub fn new() -> Self {
        Self {
            _mode: 0,
            i: 0,
            increasing: true,
            counter: 0,
            _time_counter: 0,
            _trail_speed: 50 * 5, //5 ms for key debounce
            initial: true
        } 
    }

    pub fn set_mode(&mut self, mode: u8) {
        match mode {
            0 => self.i = LOW,
            1 => self.i = LOW,
            2 => self.i = HIGH,
            3 => {
                self.i = 0;
                self.counter = 0;
                self._time_counter = 0;
                self.initial = true;
            },
            _ => (),
        }
        self._mode = mode;
    }

    pub fn get_mode(&mut self) -> u8 {
        self._mode
    }

    pub fn breathing(&mut self) {
        match self.increasing {
            true => if self.i >= HIGH {
                self.increasing = false;
            } else {
                self.i += ((( (self.i as f32) / 1500.0) as u16 ) << 1) + 1;
            },
            false => if self.i <= LOW {
                self.increasing = true;
            } else {
                self.i -= ((( (self.i as f32) / 1500.0) as u16 ) << 1) + 1;
            },
        };
    }

    pub fn harsh_blink(&mut self) {
        if self.i >= HIGH {
            self.i = LOW;
        } else {
            self.i += ((( (self.i as f32) / 1500.0) as u16 ) << 1) + 1;
        }
    }

    pub fn harsh_off(&mut self) {
        if self.i <= LOW {
            self.i = HIGH;
        } else {
            self.i -= ((( (self.i as f32) / 1500.0) as u16 ) << 1) + 1;
        }
    }

    pub fn increase_timer(&mut self) {
        self._time_counter += 1;
        if self.initial {
            self.initial = false;
        }
        if self._time_counter == self._trail_speed {
            self._time_counter = 0;
            self.counter = (self.counter + 1) % (LED_COUNT); 
        }
    }

    pub fn get_timer(&mut self) -> u16 {
        self._time_counter

    }
    
}