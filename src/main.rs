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
impl_heterogenous_array! {
    Cols,
    dyn InputPin<Error = Infallible>,
    U5,
    [0, 1, 2, 3, 4]
}

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

const my_timeout:u16 = 140;

const SF_Z: Action = HoldTap{
    timeout: my_timeout,
    hold: &k(LShift),
    tap: &k(Z)
};

const SF_SLSH: Action = HoldTap{
    timeout: my_timeout,
    hold: &k(RShift),
    tap: &k(Slash)
};

const BS_LALT: Action = HoldTap{
    timeout: my_timeout,
    hold: &k(LAlt),
    tap: &k(BSpace)
};

const SPC_CTL: Action = HoldTap{
    timeout: my_timeout,
    hold: &k(LCtrl),
    tap: &k(Space)
};

const L1_DEL: Action = HoldTap{
    timeout: my_timeout,
    hold: &l(1),
    tap: &k(Delete)
};

const L2_ENTER: Action = HoldTap{
    timeout: my_timeout,
    hold: &l(2),
    tap: &k(Enter)
};

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers = &[
    &[
        &[k(Q), k(W),  k(E),    k(R),  k(T),   k(Y),     k(U),     k(I),     k(O),   k(P)],
        &[k(A), k(S),  k(D),    k(F), k(G),    k(H),     k(J),     k(K),     k(L),   k(SColon)],
        &[SF_Z, k(X),  k(C),    k(V), k(B),    k(N),     k(M),     k(Comma), k(Dot), SF_SLSH],
        &[Trans,Trans, Trans,   BS_LALT, SPC_CTL,L1_DEL, L2_ENTER, Trans ,   Trans,  Trans],
    ], &[
        &[k(Kb1),   k(Kb2),  k(Kb3),     k(Kb4),    k(Kb5),  k(Kb6),  k(Kb7),  k(Kb8),  k(Kb9),    k(Kb0)],
        &[k(Tab),   k(No),   k(VolDown), k(VolUp),  k(No),   k(Left), k(Down), k(Up),   k(Right),  k(Quote)],
        &[k(LShift), k(No),   k(Mute),    k(No),     k(No),   k(Home), k(End),  k(PgUp), k(PgDown), k(RShift)],
        &[Trans,    Trans,   Trans,      k(LAlt),   k(Space), Trans,   k(No),   Trans,   Trans,     Trans],
    ], &[
        &[k(Escape),   k(F1), k(F2),  k(F3),  k(F4), k(No), k(No), k(No), k(Minus), k(Equal)],
        &[k(CapsLock), k(F5), k(F6),  k(F7),  k(F8), k(LBracket), k(RBracket), k(No), k(Grave), k(Bslash)],
        &[k(LShift),   k(F9), k(F10), k(F11), k(F12),k(No), k(No), k(No), k(No), k(RShift)],
        &[Trans,    Trans,  Trans,  k(LAlt),  k(SPACE), Trans,  Trans,  Trans,  Trans,  Trans,  Trans    ],
    ],
];
