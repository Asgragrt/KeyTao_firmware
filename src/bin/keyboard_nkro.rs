#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal;
use core::convert::Infallible;
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

use rp_pico as bsp;

#[entry]
fn main() -> ! {
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

    let mut keyboard = UsbHidClassBuilder::new()
        .add_device(
            usbd_human_interface_device::device::keyboard::NKROBootKeyboardConfig::default(),
        )
        .build(&usb_bus);

    //https://pid.codes
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .manufacturer("usbd-human-interface-device")
        .product("NKRO Keyboard")
        .serial_number("TEST")
        .build();

    //GPIO pins
    let mut led_pin = pins.gpio25.into_push_pull_output();

    let keys: &[&dyn InputPin<Error = core::convert::Infallible>] = &[
        &pins.gpio0.into_pull_up_input(),
        &pins.gpio1.into_pull_up_input(),
        &pins.gpio2.into_pull_up_input(),
        &pins.gpio3.into_pull_up_input(),
        &pins.gpio4.into_pull_up_input(),
        &pins.gpio5.into_pull_up_input(),
        &pins.gpio6.into_pull_up_input(),
    ];

    led_pin.set_low().ok();

    let mut input_count_down = timer.count_down();
    input_count_down.start(5.millis());

    let mut tick_count_down = timer.count_down();
    tick_count_down.start(1.millis());

    loop {
        //Poll the keys every 5ms
        if input_count_down.wait().is_ok() {
            let keys = get_keys(keys);

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

fn get_keys(keys: &[&dyn InputPin<Error = Infallible>]) -> [Keyboard; 8] {
    [
        if keys[0].is_low().unwrap() {
            Keyboard::Q
        } else {
            Keyboard::NoEventIndicated
        }, //Q
        if keys[1].is_low().unwrap() {
            Keyboard::W
        } else {
            Keyboard::NoEventIndicated
        }, //W
        if keys[2].is_low().unwrap() {
            Keyboard::E
        } else {
            Keyboard::NoEventIndicated
        }, //E
        if keys[3].is_low().unwrap() {
            Keyboard::Space
        } else {
            Keyboard::NoEventIndicated
        }, //Space
        if keys[4].is_low().unwrap() {
            Keyboard::I
        } else {
            Keyboard::NoEventIndicated
        }, //I
        if keys[5].is_low().unwrap() {
            Keyboard::O
        } else {
            Keyboard::NoEventIndicated
        }, //O
        if keys[6].is_low().unwrap() {
            Keyboard::P
        } else {
            Keyboard::NoEventIndicated
        }, //P
        if keys[6].is_low().unwrap() {
            Keyboard::U
        } else {
            Keyboard::NoEventIndicated
        }, //U
    ]
}