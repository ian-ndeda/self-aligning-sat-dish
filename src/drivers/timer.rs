use rp2040_pac::{interrupt, TIMER};
use cortex_m::peripheral::NVIC;

use crate::drivers::clocks::resets::Resets;
    
pub struct Timer {
    timer: TIMER,
}

impl Timer {
    pub fn new(timer: TIMER, resets: &Resets) -> Self {
        resets.resets.reset().modify(|_, w| w.timer().clear_bit());// Deassert timer
    
        Timer { timer }
    }
    
    pub fn enable_alarm0_interrupt(&self) {
        self.timer.inte().write(|w| w.alarm_0().set_bit());// set alarm0 interrupt
    }

    pub fn _disable_alarm0_interrupt(&self) {
        self.timer.inte().write(|w| w.alarm_0().clear_bit());// unset alarm0 interrupt
    }

    pub fn clear_alarm0_interrupt(&self) {
        self.timer.intr().write(|w| w.alarm_0().clear_bit_by_one());// clear interrupt
    }

    pub fn _unmask_interrupt(&self) {
        unsafe {
            NVIC::unmask(interrupt::TIMER_IRQ_0);// Unmask interrupt
        }
    }

    pub fn _mask_interrupt(&self) {
        NVIC::mask(interrupt::TIMER_IRQ_0);// mask interrupt
    }

    pub fn set_alarm0_ms(&self, ms: u32) {// Arg: time in ms
        let (timer_alarm0, _overflow) = (self.timer.timerawl().read().bits())
            .overflowing_add(ms * 1000); 
        
        self.timer.alarm0().write(|w|  unsafe { w
            .bits(timer_alarm0) });// set sec's after now alarm
    }
}
