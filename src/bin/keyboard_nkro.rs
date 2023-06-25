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

//Struct for easier gpio -> keypress interface
struct PinKeys<'a> {
    pin_port: &'a dyn InputPin<Error = core::convert::Infallible>,
    keys: &'a Vec<Keyboard, 4>, //Max 4 keys
}

//Macro for struct creating for each gpio - keypress
macro_rules! def_key {
    ($pin_port: expr, $keys: expr) => {
        PinKeys{
            pin_port: &$pin_port.into_pull_up_input(), 
            keys: &(Vec::from_slice(&$keys).unwrap())}          
    };
}
//Keys used
const KEY_COUNT: usize = 8;

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

    let sio = hal::Sio::new(pac.SIO);
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
    let pin_build: &[PinKeys; KEY_COUNT] = &[
        def_key!(pins.gpio0, [Keyboard::Q]),
        def_key!(pins.gpio1, [Keyboard::W]),
        def_key!(pins.gpio2, [Keyboard::E]),
        def_key!(pins.gpio3, [Keyboard::Space]),
        def_key!(pins.gpio4, [Keyboard::I]),
        def_key!(pins.gpio5, [Keyboard::O]),
        def_key!(pins.gpio6, [Keyboard::P]),
        def_key!(pins.gpio7, [Keyboard::LeftControl, Keyboard::O]),
    ];

    led_pin.set_low().ok();

    let mut input_count_down = timer.count_down();
    input_count_down.start(1.millis());

    let mut tick_count_down = timer.count_down();
    tick_count_down.start(1.millis());

    loop {
        //Poll the keys every 1ms
        if input_count_down.wait().is_ok() {
            let keys = get_keys(pin_build);

            match keyboard.device().write_report(keys) {
                Err(UsbHidError::WouldBlock) => {}
                Err(UsbHidError::Duplicate) => {}
                Ok(_) => {}
                Err(e) => {
                    core::panic!("Failed to write keyboard report: {:?}", e)
                }
            };
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

fn get_keys(pins: &[PinKeys]) -> Vec<Keyboard, KEY_COUNT>{
    let mut key_return: Vec<Keyboard, KEY_COUNT> = Vec::new();
    for i in 0..KEY_COUNT {
        if pins[i].pin_port.is_low().unwrap() {
            key_return.extend(pins[i].keys.clone());
        }
    }
    return key_return;
}