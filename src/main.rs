#![no_main]
#![no_std]

//panic handler
use panic_halt as _;

use core::convert::Infallible;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use generic_array::typenum::{U4, U5};
use atsamd_hal as hal;
use hal::gpio::{v2,Floating, Input, Output, PullUp, PushPull};
use hal::gpio::v2::Pin;
use hal::prelude::*;
use hal::sercom;
use hal::clock;
use nb::block;

//keyberon imports
use keyberon::action::{k, l, m, Action, Action::*};
use keyberon::debounce::Debouncer;
use keyberon::impl_heterogenous_array;
use keyberon::key_code::KbHidReport;
use keyberon::key_code::KeyCode::*;
use keyberon::layout::{Event, Layout};
use keyberon::matrix::{Matrix, PressedKeys};

use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass as _;
use usb_device::device::UsbDeviceState;

//type UsbClass = keyberon::Class<'static, upe>;

trait ResultExt<T>{
    fn get(self) -> T;
}

impl<T> ResultExt<T> for Result<T, Infallible>{
    fn get(self) -> T{
        match self{
            Ok(v) => v,
            Err(e) => match e{},
        }
    }
}

pub struct Cols(
    Pin<v2::PA02,Input<PullUp>>,
    Pin<v2::PA04,Input<PullUp>>,
    Pin<v2::PA10,Input<PullUp>>,
    Pin<v2::PA11,Input<PullUp>>,
    Pin<v2::PA09,Input<PullUp>>,
);
//this here doesn't work
// impl_heterogenous_array! {
//     Cols,
//     dyn InputPin<Error = Infallible>,
//     U5,
//     [0, 1, 2, 3, 4]
// }

pub struct Rows(
    Pin<v2::PA06,Output<PushPull>>,
    Pin<v2::PA05,Output<PushPull>>,
    Pin<v2::PA07,Output<PushPull>>,
    Pin<v2::PA08,Output<PushPull>>,
);
impl_heterogenous_array! {
    Rows,
    dyn OutputPin<Error = Infallible>,
    U4,
    [0, 1, 2, 3]
}

// const SF_Z: Action = HoldTap{
//     timeout: 200,
//     hold: &k()
// }
