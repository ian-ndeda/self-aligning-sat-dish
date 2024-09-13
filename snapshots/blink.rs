#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use rp2040_pac::interrupt;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use rp2040_pac::{self as pac, Interrupt, SIO, TIMER};
use cortex_m::peripheral::NVIC;

// Global variable for peripherals
static TIMER: Mutex<RefCell<Option<TIMER>>> = Mutex::new(RefCell::new(None));
static SIO: Mutex<RefCell<Option<SIO>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let dp = unsafe { pac::Peripherals::steal() };// Take the device peripherals
    let _cp = pac::CorePeripherals::take().unwrap();// Take the core peripherals

    // Configure the External Oscillator
    let xosc = dp.XOSC;
    xosc.ctrl().modify(|_, w| w.freq_range()._1_15mhz());// Set freq. range
    xosc.startup().write(|w| unsafe { w.delay().bits(47) });// Set the startup delay
    xosc.ctrl().modify(|_, w| w.enable().enable());// Enable the xosc
    while xosc.status().read().stable().bit_is_clear() {}// Await stabilization

    // Clocks configuration
    let clocks = dp.CLOCKS;
    let pll_sys = dp.PLL_SYS;
    let resets = dp.RESETS;

    resets.reset().modify(|_, w| w
                  .pll_sys().clear_bit()// deassert pll sys
                  .busctrl().clear_bit()// deassert bus ctrl??
                  );
    pll_sys.pwr().reset();// Turn off PLL in case it is already running
    pll_sys.fbdiv_int().reset();
    pll_sys.cs().modify(|_, w| unsafe { w.refdiv().bits(1) });// Set refdiv as 1
    pll_sys.fbdiv_int().write(|w| unsafe { w.bits(125) });// Set fbdiv_int as 125
    pll_sys.pwr().modify(|_, w| w
                       .pd().clear_bit()// Turn on PLL
                       .vcopd().clear_bit()// Turn on VCO
                       );
    while pll_sys.cs().read().lock().bit_is_clear() {}// Await locking of pll
    pll_sys.prim().modify(|_, w| unsafe {
        w.postdiv1().bits(6)// Set up postdivider 1
            .postdiv2().bits(2)// Set up postdivider 2
    });
    pll_sys.pwr().modify(|_, w| w.postdivpd().clear_bit());// Turn on postdividers

    // Select ref clock source as XOSC divided by 1
    clocks.clk_ref_ctrl().modify(|_, w| w.src().xosc_clksrc());
    clocks.clk_ref_div().modify(|_, w| unsafe { w.int().bits(1) });

    // Set pll sys as clk sys
    clocks.clk_sys_ctrl().modify(|_, w| w.src().clksrc_clk_sys_aux());
    clocks.clk_sys_div().modify(|_, w| unsafe { w.int().bits(1) });

    // Set clk sys as peripheral clock
    // Used as reference clock for Peripherals
    clocks.clk_peri_ctrl().modify(|_, w| w
                                .auxsrc().clk_sys()
                                .enable().set_bit()
                                );

    // Watchdog: to provide the clk_tick required by the timer
    let watchdog = dp.WATCHDOG;
    watchdog.tick().modify(|_, w| unsafe{ w
        .cycles().bits(12)// For an effective frequency of 1MHz
        .enable().set_bit()
    });

    // Timer set up
    let timer = dp.TIMER;
    resets.reset().modify(|_, w| w.timer().clear_bit());// Deassert timer
    timer.inte().modify(|_, w| w.alarm_0().set_bit());// set alarm0 interrupt
    unsafe {
        NVIC::unmask(Interrupt::TIMER_IRQ_0);// Unmask interrupt
    }
    timer.alarm0().modify(|_, w| unsafe{ w.bits(timer.timerawl().read().bits() + 1_000_000) });// set 1 sec after now alarm

    // Set up GPIO pins
    let sio = dp.SIO;
    let io_bank0 = dp.IO_BANK0;
    let pads_bank0 = dp.PADS_BANK0;

    resets.reset().modify(|_, w| w
                        .busctrl().clear_bit()
                        .io_bank0().clear_bit()
                        .pads_bank0().clear_bit());

    // GP25
    pads_bank0.gpio(25).modify(|_, w| w
                               .pue().set_bit()// pull up enable
                               .pde().set_bit()// pull down enable
                               .od().clear_bit()// output disable
                               .ie().set_bit()// input enable
                               );
    io_bank0.gpio(25).gpio_ctrl().modify(|_, w| w.funcsel().sio());// set function as sio 
    sio.gpio_oe().modify(|r, w| unsafe { w.bits(r.gpio_oe().bits() | 1 << 25)});// Output enable for pin 25

    // Move peripherals into global scope
    cortex_m::interrupt::free(|cs| {
        SIO.borrow(cs).replace(Some(sio));
        TIMER.borrow(cs).replace(Some(timer));
    });

    loop {
        cortex_m::asm::wfi();// wait for interrupt
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    // Enter critical section
    cortex_m::interrupt::free(|cs| {
    // Borrow the peripherals under critical section. 
        let mut timer = TIMER.borrow(cs).borrow_mut();
        let mut sio = SIO.borrow(cs).borrow_mut();

        timer.as_mut().unwrap().intr().modify(|_, w| w.alarm_0().clear_bit_by_one());// first clear interrupt
        sio.as_mut().unwrap().gpio_out().modify(|r, w| unsafe { w.bits(r.gpio_out().bits() ^ 1 << 25)});// toggle bit 25    
        // set period for next alarm
        let (timer_alarm0, _overflow) = (timer.as_mut().unwrap().timerawl().read().bits())
            .overflowing_add(1_000_000); 

        timer.as_mut().unwrap().alarm0().write(|w|  unsafe { w
            .bits(timer_alarm0) });// set sec's after now alarm
    });
}