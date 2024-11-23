use rp2040_pac::PWM;

use crate::drivers::{clocks::resets::Resets, gpio::gpios::{Pin, typestate::{Output, FunctionPwm, PushPull}}};

pub struct Pwm<const INDEX: usize>;

impl<const INDEX: usize> Pwm<INDEX> {
    fn new() -> Self { Self }

    pub fn configure_50_hz(&self, _: Pin<Output<FunctionPwm<PushPull>>, 20>) -> Self {
        const WRAP_VALUE: u32 = 49999;
        unsafe {
            (*PWM::ptr()).ch(INDEX).csr().modify(|_, w| w
                             .divmode().div()//free runnning counter
                             .ph_correct().set_bit()
                             );
            (*PWM::ptr()).ch(INDEX).div().modify(|_, w| w.int().bits(25).frac().bits(0) );//For a fpwm of 50Hz w/ top of 50000
            (*PWM::ptr()).ch(INDEX).top().modify(|_, w| w.bits(WRAP_VALUE) );//wrap value
        }

        Self
    }

    pub fn enable(&self) {
        unsafe {
            (*PWM::ptr()).ch(INDEX).csr().modify(|_, w| w.en().set_bit());//Enable channel 1
        }
    }
    
    pub fn sweep(&self, mut degrees: u16) {
        const BASE: u16 = 1400;// here pwm at position 0 degrees

        degrees = BASE + (degrees * 28);// ~28: movt by 1 degree 
        unsafe {
            (*PWM::ptr()).ch(INDEX).cc().modify(|_, w| w.a().bits(degrees) );// move by one degree * no of degrees
        }
    }
}

pub struct PwmSlices {
    pub pwm2: Pwm<2>
    //...add pwm slices
}

impl PwmSlices {
    pub fn new(_: PWM, resets: &Resets) -> Self {
        resets.resets.reset().modify(|_, w| w.pwm().clear_bit());
        
        Self {
            pwm2: Pwm::new(),
        }
    }
}
