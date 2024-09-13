#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use core::{
    f32::consts::PI,
    cell::{Cell, RefCell},
    fmt::Write,
};
use cortex_m::interrupt::Mutex;
use rp2040_pac::{interrupt, UART0, UART1};

use heapless::{Vec, String};
use micromath::F32Ext;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use rp2040_pac::{self as pac, Interrupt, SIO, TIMER};

// Global variable for peripherals
static TIMER: Mutex<RefCell<Option<TIMER>>> = Mutex::new(RefCell::new(None));
static SIO: Mutex<RefCell<Option<SIO>>> = Mutex::new(RefCell::new(None));
static UARTCMD: Mutex<RefCell<Option<UART1>>> = Mutex::new(RefCell::new(None));
static UARTDATA: Mutex<RefCell<Option<UART0>>> = Mutex::new(RefCell::new(None));

// Data
static DIRECTION: Mutex<RefCell<Option<Direction>>> = Mutex::new(RefCell::new(None));
static BUFFER: Mutex<RefCell<Option<String<164>>>> = Mutex::new(RefCell::new(None));// buffer to hold received gps data

// Flags
static AUTO: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));
static AUTO_FIRST_ITER: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));
static STARTED: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));
static GGA_CONFIRMED: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));
static GGA_ACQUIRED: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

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

    // Configuring UART1; command reception
    let uart_cmd = dp.UART1;
    resets.reset().modify(|_, w| w.uart1().clear_bit());// Deassert uart_cmd
    // Set baudrate at 96 00
    uart_cmd.uartibrd().write(|w| unsafe { w.bits(813)});
    uart_cmd.uartfbrd().write(|w| unsafe { w.bits(51)});
    uart_cmd.uartlcr_h().modify(|_, w| unsafe { w
            .wlen().bits(0b11)// Set word length as 8
    });
    uart_cmd.uartcr().modify(|_, w| w
                        .uarten().set_bit()// Enable uart_cmd
                        .txe().set_bit()// Enable tx
                        .rxe().set_bit()// Enable rx
                        );
    uart_cmd.uartimsc().modify(|_, w| w
                .rtim().set_bit());// set interrupt for when there's one byte in uart rx fifo

    // Configuring UART0; GPS data reception
    let uart_data = dp.UART0;
    resets.reset().modify(|_, w| w.uart0().clear_bit());// Deassert uart_data
    // Set baudrate at 96 00
    uart_data.uartibrd().write(|w| unsafe { w.bits(813)});
    uart_data.uartfbrd().write(|w| unsafe { w.bits(51)});
    uart_data.uartlcr_h().modify(|_, w| unsafe { w
    .fen().set_bit()// Enable FIFO
            .wlen().bits(0b11)// Set word length as 8
    });
    uart_data.uartcr().modify(|_, w| w
                        .uarten().set_bit()// Enable uart_data
                        .txe().set_bit()// Enable tx
                        .rxe().set_bit()// Enable rx
                        );
    uart_data.uartimsc().modify(|_, w| w
                               .rxim().set_bit());// set interrupt for when there data in rx fifo

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

    // Configure gp8 as UART1 Tx Pin
    pads_bank0.gpio(8).modify(|_, w| w
                               .pue().set_bit()
                               .pde().set_bit()
                               .od().clear_bit()
                               .ie().set_bit()
                               );

    io_bank0.gpio(8).gpio_ctrl().modify(|_, w| w.funcsel().uart());

    // Configure gp9 as UART1 Rx Pin
    // Will be connected to HC-05 Tx Pin
    pads_bank0.gpio(9).modify(|_, w| w
                               .pue().set_bit()
                               .pde().set_bit()
                               .od().clear_bit()
                               .ie().set_bit()
                               );

    io_bank0.gpio(9).gpio_ctrl().modify(|_, w| w.funcsel().uart());

    // Configure gp0 as UART0 Tx Pin
    // Connected to HC-05 Rx Pin
    pads_bank0.gpio(0).modify(|_, w| w
                               .pue().set_bit()
                               .pde().set_bit()
                               .od().clear_bit()
                               .ie().set_bit()
                               );

    io_bank0.gpio(0).gpio_ctrl().modify(|_, w| w.funcsel().uart());

    // Configure gp1 as UART0 Rx Pin
    // Will be connected to GPS Module Tx Pin
    pads_bank0.gpio(1).modify(|_, w| w
                               .pue().set_bit()
                               .pde().set_bit()
                               .od().clear_bit()
                               .ie().set_bit()
                               );

    io_bank0.gpio(1).gpio_ctrl().modify(|_, w| w.funcsel().uart());


    // Buffers
    let gpsbuf = String::new();// buffer to hold gps data
    let serialbuf = &mut String::<164>::new();// buffer to hold data to be serially transmitted

    // Variables
    let (mut theta, mut phi) = (0., 0.);

    // Tests
    writeln!(serialbuf, "\nUart command test.").unwrap();
    transmit_uart_cmd(&uart_cmd, serialbuf);

    writeln!(serialbuf, "\nUart data test.\n").unwrap();
    transmit_uart_data(&uart_data, serialbuf);

    let mut position = Position::new();// gps position struct

    // Move peripherals into global scope
    cortex_m::interrupt::free(|cs| {
        SIO.borrow(cs).replace(Some(sio));
        TIMER.borrow(cs).replace(Some(timer));
        BUFFER.borrow(cs).replace(Some(gpsbuf));
        UARTCMD.borrow(cs).replace(Some(uart_cmd));
        UARTDATA.borrow(cs).replace(Some(uart_data));
    });

    // Unmask interrupts
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIMER_IRQ_0);
        cortex_m::peripheral::NVIC::unmask(interrupt::UART0_IRQ);
        cortex_m::peripheral::NVIC::unmask(interrupt::UART1_IRQ);
    }

    loop {
        cortex_m::interrupt::free(|cs| {
            let mut uart_data = UARTDATA.borrow(cs).borrow_mut();
            let mut buffer = BUFFER.borrow(cs).borrow_mut();

            if GGA_ACQUIRED.borrow(cs).get() {
                // Transmit raw data from gps
                write!(serialbuf, "\n{}", buffer.as_mut().unwrap()).unwrap(); 
                transmit_uart_data(
                    uart_data.as_mut().unwrap(),
                    serialbuf);
                GGA_ACQUIRED.borrow(cs).set(false);// clear the flag

                // update the coordinates from the nmea sentence
                parse_gga(buffer.as_mut().unwrap(), &mut position);

                // Transmit gps position
                let lat = position.lat;
                let long = position.long;
                writeln!(serialbuf, "lat: {}  long: {}", lat, long).unwrap();
                transmit_uart_data(
                    uart_data.as_mut().unwrap(),
                    serialbuf);

                // Calculate look angles
                (theta, phi) = get_look_angles(
                    position.lat,
                    position.long,
                    position.alt);

                writeln!(serialbuf, "theta: {}  phi: {}", theta, phi).unwrap();
                transmit_uart_data(
                    uart_data.as_mut().unwrap(),
                    serialbuf);

            }
        });

        cortex_m::asm::wfi();// wait for interrupt
    }
}

fn receive_uart_cmd(uart: &UART1) -> u8 {// receive 1 byte
    while uart.uartfr().read().rxfe().bit_is_set() {} // wait until byte received

    uart.uartdr().read().data().bits()// Received data
}

fn transmit_uart_cmd(uart: &UART1, buffer: &mut String<164>) {
    for ch in buffer.chars() {
        uart.uartdr().modify(|_, w| unsafe { w.data().bits(ch as u8)});// Send data
        while uart.uartfr().read().busy().bit_is_set()  {}// Wait until tx finished
    }

    buffer.clear()
}

fn receive_uart_data(uart: &UART0) -> u8 {// receive 1 byte
    while uart.uartfr().read().rxfe().bit_is_set() {} // wait until byte received  

    uart.uartdr().read().data().bits()// Received data
}

fn transmit_uart_data(uart: &UART0, buffer: &mut String<164>) {
    for ch in buffer.chars() {
        uart.uartdr().modify(|_, w| unsafe { w.data().bits(ch as u8)});// Send data
        while uart.uartfr().read().txfe().bit_is_clear()  {}// Wait until tx finished
    }

    buffer.clear()
}

fn parse_gga(buffer: &mut String<164>, position: &mut Position) {
    //  Updates position of earth station
    let v : Vec<&str, 84> = buffer.split_terminator(',').collect();

    if !v[2].is_empty() {// latitude
        let (x, y) = v[2].split_at(2);

        let x_f32 = x.parse::<f32>().unwrap_or(0.);

        let y_f32 = y.parse::<f32>().unwrap_or(0.);

        if x_f32 + (y_f32/60.) != 0. {// if reading valid i.e. != 0 update position
            position.lat = x_f32 + (y_f32/60.);

            if v[3] == "S" {
                position.lat *= -1.;
            }
        }

        if !v[4].is_empty() {
            let (a, b) = v[4].split_at(3);

            let a_f32 = a.parse::<f32>().unwrap_or(0.);

            let b_f32 = b.parse::<f32>().unwrap_or(0.);

            if a_f32 + (b_f32/60.) != 0. {
                position.long = a_f32 + (b_f32/60.);// if reading valid i.e. != 0 update position

                if v[5] == "W" {
                    position.long *= -1.;
                }
            }

            if !v[9].is_empty()  {
                let c = v[9];
                position.alt = match c.parse::<f32>() {
                    Ok(c) => c,
                    Err(_) => position.lat,// replace w/ last position
                };
            }
        }
    }
}

fn get_look_angles(lat: f32, long: f32, alt: f32) -> (f32, f32) {
    // Earth Station in radians
    let le: f32 = lat*PI/180.; // Latitude: N +ve, S -ve??
    let ie: f32 = long*PI/180.; // Longitude: E +ve, W -ve??
    let _h: f32 = alt; // Altitude

    // Satellite position in radians
    const _LS: f32 = 0.;// Latitude
    const IS: f32 = 50.*PI/180.;// Longitude

    // Determine Differential Longitude, b
    let b = ie - IS;

    let mut theta = (((le.cos()*b.cos())-0.151)/(1.-(le.cos()*le.cos()*b.cos()*b.cos())).sqrt()).atan();
    let mut ai = (b.tan()/le.sin()).atan();

    // Convert into Deg.
    theta *= 180.0/PI;
    ai *= 180./PI;

    // Azimuth Angle
    let phi_z = {
        if (le < 0.) && (b > 0.) {
            360. - ai
        } else if (le > 0.) && (b < 0.) {
            180. + ai
        } else if (le > 0.) && (b > 0.) {
            180. - ai
        } else {
            ai
        }
    };

    (theta, phi_z)
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

#[interrupt]
fn UART1_IRQ() {
    // Enter critical section
    cortex_m::interrupt::free(|cs| {
        // peripheral handles
        let mut uart = UARTCMD.borrow(cs).borrow_mut();
        let mut dir = DIRECTION.borrow(cs).borrow_mut();

        uart.as_mut().unwrap().uarticr().modify(|_, w| w.rtic().bit(true));// clear rx interrupt 

        // receive commands
        let b = receive_uart_cmd(uart.as_mut().unwrap() );// Received byte

        if b == b'A' { // toggle between auto and manual modes
            if AUTO.borrow(cs).get() {
                AUTO.borrow(cs).set(false);
            } else {
                AUTO.borrow(cs).set(true);
                // if toggling to auto; set first iter true
                AUTO_FIRST_ITER.borrow(cs).set(true);
            }
        } else {
            // update direction cmd only in manual mode
            if !AUTO.borrow(cs).get() {
                if b == b'B' {
                    *dir = Some(Direction::Cw);
                } else if b == b'C' {
                    *dir = Some(Direction::Ccw);
                } else if b == b'D' {
                    *dir = Some(Direction::Up);
                } else if b == b'E' {
                    *dir = Some(Direction::Down);
                } else if b == b'F' {
                    *dir = Some(Direction::Zero);
                }
            }
        }
    });
}

#[interrupt]
fn UART0_IRQ() {
    // Enter critical section
    cortex_m::interrupt::free(|cs| {
        // Peripherals
        let mut uart = UARTDATA.borrow(cs).borrow_mut();
        // Flags
        let started = STARTED.borrow(cs);
        let gga_confirmed = GGA_CONFIRMED.borrow(cs);
        let gga_acquired = GGA_ACQUIRED.borrow(cs);

        let mut buffer = BUFFER.borrow(cs).borrow_mut();

        uart.as_mut().unwrap().uarticr().modify(|_, w| w.rxic().bit(true));// clear rx interrupt

        // Data acquisition
        if started.get() {// If nmea sentence acquisition started
            if gga_confirmed.get() {
                let b = receive_uart_data(uart.as_mut().unwrap());// Received data
                buffer.as_mut().unwrap().push(char::from(b)).unwrap();// push to buffer
                if b == b'\n' {// End of nmea sentence
                    gga_acquired.set(true);// set flag
                    gga_confirmed.set(false);// unset confirmed flag
                    started.set(false);// unset flag; start again
                }
            } else {
                let b = receive_uart_data(uart.as_mut().unwrap());// Received data
                buffer.as_mut().unwrap().push(char::from(b)).unwrap();// push to buffer
                if buffer.as_mut().unwrap().len() == 6 {// at len of 6 check if gga
                    if buffer.as_mut().unwrap() == "$GPGGA" {// check if gga
                        gga_confirmed.set(true);// set flag
                    } else {
                        started.set(false);// unset flag; start again
                    }
                }
            }
        } else {
            buffer.as_mut().unwrap().clear();// clear buffer
            gga_acquired.set(false);// service flag incase not read; to avoid reading empty buffer
            let b = receive_uart_data(uart.as_mut().unwrap());// Received data
            if b == b'$'{//start of nmea sentence
                buffer.as_mut().unwrap().push(char::from(b)).unwrap();// push to buffer
                started.set(true);
            }
        }
    });
}

enum Direction {
    Cw,
    Ccw,
    Up,
    Down,
    Zero,
}

struct Position {
    lat: f32,
    long: f32,
    alt: f32,
}

impl Position {
    fn new() -> Self {
        Position {
            lat: 0.,
            long: 0.,
            alt: 0.,
        }
    }
}
