#![no_std]
use usbd_human_interface_device::page::Keyboard;
use embedded_hal::digital::v2::InputPin;
use heapless::Vec;
pub use core::cmp::min;

//Keys used
pub const KEY_COUNT: usize = 9;

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

//At least 2 keys and not your main ones
pub fn get_keys(pins: &[PinKeys]) -> (Vec<Keyboard, KEY_COUNT>, bool){
    let mut key_return: Vec<Keyboard, KEY_COUNT> = Vec::new();
    let mut limit: usize = KEY_COUNT;
    let led_change: bool = pins[KEY_COUNT - 2].get_pin().is_low().unwrap() && pins[KEY_COUNT - 1].get_pin().is_low().unwrap();
    //Branchless optimization
    limit -= 2 * (led_change as usize);
    for i in 0..limit {
        if pins[i].get_pin().is_low().unwrap() {
            key_return.extend(pins[i].get_keys().clone());
        }
    }
    return (key_return, led_change);
}

/*********LED CONTROL*/

pub const LOW: u16 = 0;
pub const HIGH: u16 = 55_000;

pub const LED_COUNT: i8 = 10;
pub const MODE_COUNT: u8 = 8;

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
            3 | 4 | 5 => {
                let mut led_count: i8 = 0;
                
                $(
                    $pwm_channel_ident.set_duty(
                        $pin_modes.displacement_duty(
                            led_count, $pwm_channel_ident.get_duty()
                        )
                    );                   
                    
                    led_count += 1;
                )*
                $pin_modes.increase_timer();
            },
            6 | 7 => {
                $(
                    $pwm_channel_ident.set_duty($pin_modes.i);
                )*                
            },
            _ => (),
        }
    };
}

pub fn breath_approx(x: i8) -> f32 {
    match x {
        0 => 1.0,
        1 => 0.3,
        2 => 0.05,
        _ => 0.0,
    }
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
    pub time_counter: u16,
    pub trail_speed: u16,
}

impl PinModes {
    pub fn new() -> Self {
        Self {
            _mode: 0,
            i: 0,
            increasing: true,
            counter: 0,
            time_counter: 0,
            trail_speed: 250, //50 * 5, 5 ms for key debounce
        } 
    }

    pub fn set_mode(&mut self, mode: u8) {
        match mode {
            0 | 1 | 6 => self.i = LOW + 1,
            2 | 7     => self.i = HIGH,
            3 | 4 | 5 => {
                self.counter = 0;
                self.time_counter = 0;
                //Branchless optimization
                self.trail_speed = 300 * ((mode == 5) as u16) 
                                + 250 * ((mode != 5) as u16);
            },
            _ => {
                self._mode = 0;
                self.i = LOW;
            },
        }
        self._mode = mode;
    }

    pub fn get_mode(&self) -> u8 {
        self._mode
    }

    pub fn increase_mode(&mut self) {
        self.set_mode((self.get_mode() + 1) % MODE_COUNT)
    }

    pub fn breathing(&mut self) {
        let difference: u16 = ((( (self.i as f32) / 1500.0) as u16 ) << 1) + 1;
        //Branchless optimization
        self.increasing = self.increasing ^ (self.i > HIGH || self.i < LOW + 1);
        self.i = self.i + difference * (self.increasing as u16) 
                - difference * (!self.increasing as u16);
    }

    pub fn harsh_blink(&mut self) {  
        //Branchless optimization      
        self.i = (self.i * (self.i < HIGH) as u16) 
                + ((( (self.i as f32) / 1500.0) as u16 ) << 1) + 1;
    }

    pub fn harsh_off(&mut self) {
        //Branchless optimization
        self.i = (self.i * (self.i > LOW) as u16) + (HIGH * (self.i <= LOW) as u16)
                - (((( (self.i as f32) / 1500.0) as u16 ) << 1) + 1);

    }

    pub fn increase_timer(&mut self) {
        self.time_counter += 1;
        if self.time_counter == self.trail_speed {
            self.time_counter = 0;
            self.counter = (self.counter + 1) % (LED_COUNT); 
        }
    }

    fn get_timer_position(&self) -> u16{
        self.trail_speed - self.time_counter
    }

    fn breathing_displacement(&mut self, ext_counter: i8) -> i8{
        //Pin modes counter = counter_1, led_count = counter_2
        let mut dif: i8 = (self.counter - ext_counter).abs();
        match self.get_mode() {
            3 => {
                min(dif, (LED_COUNT - dif).abs())
            },
            4 => {
                min(dif, (LED_COUNT - 1 
                    - self.counter - ext_counter).abs())
            },
            5 | _ => {
                if ext_counter < LED_COUNT / 2 + LED_COUNT % 2 {
                    min(dif, (LED_COUNT - dif).abs())
                }
                else {
                    dif = (LED_COUNT - 1 - self.counter - ext_counter).abs();
                    min(dif, (LED_COUNT - dif).abs())
                }
            }
        }
    }
    pub fn displacement_duty(&mut self, led_count: i8, duty: u16) -> u16 {
        let val: u16 = (HIGH as f32 * breath_approx(self.breathing_displacement(led_count)) ) as u16;
        linear(duty, val, self.get_timer_position())
    }
    
}