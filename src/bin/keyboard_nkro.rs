#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal;
//use core::convert::Infallible;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::*;
use embedded_hal::prelude::*;
use fugit::ExtU32;
use hal::pac;
use panic_probe as _;
#[allow(clippy::wildcard_imports)]
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_human_interface_device::page::Keyboard;
use usbd_human_interface_device::prelude::*;
use usbd_human_interface_device::device::keyboard::{NKROBootKeyboardConfig, NKRO_BOOT_KEYBOARD_REPORT_DESCRIPTOR};
use usbd_human_interface_device::usb_class::prelude::{ManagedIdleInterfaceConfig, InterfaceBuilder};
use usbd_human_interface_device::descriptor::InterfaceProtocol;
use rp_pico as bsp;
use heapless::Vec;
use hal::multicore::{Multicore, Stack};
use firmware::*;

const SYS_FREQ: u32 = 12_000_000;

static mut CORE1_STACK: Stack<4096> = Stack::new();

#[entry]
fn main() -> ! {
    //Basic mcu control related interfaces
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    let mut sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("Starting");

    //USB
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        let mut pac = unsafe { pac::Peripherals::steal() };
        let core = unsafe { pac::CorePeripherals::steal() };

        let mut sio = hal::Sio::new(pac.SIO);

        let mut led_pin = pins.gpio16.into_push_pull_output();
        let led_button  = pins.gpio17.into_pull_up_input();

        let mut delay = cortex_m::delay::Delay::new(core.SYST, SYS_FREQ);

        let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

        //Generates the necessary pwm setup for each pin
        
        def_pwm!(pwm_slices, 
            [pwm1_slice, pwm1, [pins.gpio18, channel_a, pwm1_a;
                                pins.gpio19, channel_b, pwm1_b];
            pwm2_slice, pwm2,  [pins.gpio20, channel_a, pwm2_a;
                                pins.gpio21, channel_b, pwm2_b];
            pwm3_slice, pwm3,  [pins.gpio22, channel_a, pwm3_a];
            pwm5_slice, pwm5,  [pins.gpio26, channel_a, pwm5_a;
                                pins.gpio27, channel_b, pwm5_b]]);

        let mut pin_modes = PinModes::new();

        led_pin.set_low().ok();
        let mut prev: bool;
        

        loop {
            prev = led_button.is_high().unwrap();
            delay.delay_ms(5);

            let change_led = sio.fifo.read();
            
            if let Some(1) = change_led {
                pin_modes.increase_mode();
                delay.delay_ms(1500);
            }
            sio.fifo.drain();

            if prev && led_button.is_low().unwrap() {
                led_pin.toggle().ok();
                pin_modes.increase_mode();
            }
            pwm_mode!(pin_modes, [pwm1_a, pwm1_b, pwm2_a, pwm2_b, pwm3_a, pwm5_a, pwm5_b]);
            
        }
    });

    //Custon keyboard config for higher polling rate
    let config = NKROBootKeyboardConfig::new(ManagedIdleInterfaceConfig::new(
        unwrap!(unwrap!(unwrap!(unwrap!(InterfaceBuilder::new(
            NKRO_BOOT_KEYBOARD_REPORT_DESCRIPTOR
        ))
        .description("NKRO Keyboard")
        .boot_device(InterfaceProtocol::Keyboard)
        .idle_default(500.millis()))
        .in_endpoint(1.millis()))
        .with_out_endpoint(100.millis()))
        .build(),
    ));

    let mut keyboard = UsbHidClassBuilder::new().add_device(config).build(&usb_bus);

    //https://pid.codes
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .manufacturer("usbd-human-interface-device")
        .product("NKRO Keyboard")
        .serial_number("TEST")
        .build();

    //Status led pin
    let mut led_pin = pins.gpio25.into_push_pull_output();

    //GPIO -> Keypress relations
    let (pins, keys): ([&dyn InputPin<Error = core::convert::Infallible>; KEY_COUNT], 
        [&Vec<Keyboard, 4>; KEY_COUNT]) = pin_keys!(
        pins.gpio0, [Keyboard::Q];
        pins.gpio1, [Keyboard::W];
        pins.gpio2, [Keyboard::E];
        pins.gpio3, [Keyboard::Space];
        pins.gpio4, [Keyboard::I];
        pins.gpio5, [Keyboard::O];
        pins.gpio6, [Keyboard::P];
        pins.gpio7, [Keyboard::LeftControl, Keyboard::O];
        pins.gpio8, [Keyboard::F1]);

    let pin_build = &get_pin_keys(pins, keys);      

    led_pin.set_low().ok();

    let mut input_count_down = timer.count_down();
    input_count_down.start(1.millis());

    let mut tick_count_down = timer.count_down();
    tick_count_down.start(1.millis());

    loop {
        //Poll the keys every 1ms
        if input_count_down.wait().is_ok() {
            let (keys, change_mode) = get_keys(pin_build);

            match keyboard.device().write_report(keys) {
                Err(UsbHidError::WouldBlock) => {}
                Err(UsbHidError::Duplicate) => {}
                Ok(_) => {}
                Err(e) => {
                    core::panic!("Failed to write keyboard report: {:?}", e)
                }
            };

            if !sio.fifo.is_read_ready() {
                sio.fifo.write(change_mode as u32);
            }
        }

        //Tick once per ms
        if tick_count_down.wait().is_ok() {
            match keyboard.tick() {
                Err(UsbHidError::WouldBlock) => {}
                Ok(_) => {}
                Err(e) => {
                    core::panic!("Failed to process keyboard tick: {:?}", e)
                }
            };
        }

        if usb_dev.poll(&mut [&mut keyboard]) {
            match keyboard.device().read_report() {
                Err(UsbError::WouldBlock) => {
                    //do nothing
                }
                Err(e) => {
                    core::panic!("Failed to read keyboard report: {:?}", e)
                }
                Ok(leds) => {
                    led_pin.set_state(PinState::from(leds.num_lock)).ok();
                }
            }
        }
    }
}

//At least 3 keys
fn get_keys(pins: &[PinKeys]) -> (Vec<Keyboard, KEY_COUNT>, bool){
    let mut key_return: Vec<Keyboard, KEY_COUNT> = Vec::new();
    let mut limit: usize = KEY_COUNT;
    let led_change: bool = pins[KEY_COUNT - 2].get_pin().is_low().unwrap() && pins[KEY_COUNT - 1].get_pin().is_low().unwrap();
    if  led_change {
        limit -= 2;
    }
    for i in 0..limit {
        if pins[i].get_pin().is_low().unwrap() {
            key_return.extend(pins[i].get_keys().clone());
        }
    }
    return (key_return, led_change);
}