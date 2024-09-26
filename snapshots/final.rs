#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use core::{
    f32::consts::PI,
    cell::{Cell, RefCell},
    fmt::Write,
};

use cortex_m::{
    delay::Delay,
    interrupt::Mutex
};

use heapless::{Vec, String};
use micromath::F32Ext;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use rp2040_pac::{
    self as pac,
    pwm::ch::CH,
    Interrupt, interrupt,
    UART0, UART1, I2C0, SIO, TIMER
};

// Global constants
const TX_FIFO_SIZE: u8 = 16;// I2C FIFO size

// Slave Address
const HMC5883L_ADDR: u16 = 30;

// Mode Register.
const MODE_R: u8 = 0x02; // HMC5883L

// Configuration Register A
const CFG_REG_A: u8 = 0x0;// HMC5883L

// Configuration Register B
const CFG_REG_B: u8 = 0x01;// HMC5883L

// Data Registers.
const DATA_OUTPUT_X_MSB_R: u8 = 0x03;// HMC5883L

// ID
const IDENTIFICATION_REG_A: u8 = 0xA;// HMC5883L
const IDENTIFICATION_REG_B: u8 = 0xB;// HMC5883L
const IDENTIFICATION_REG_C: u8 = 0xC;// HMC5883L

const MAGNETIC_DECLINATION: f32 = 1.667;

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
    let cp = pac::CorePeripherals::take().unwrap();// Take the core peripherals

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

    // Set up I2C
    let mut i2c0 = dp.I2C0;
    resets.reset().modify(|_,w| w.i2c0().clear_bit());

    i2c0.ic_enable().modify(|_, w| w.enable().clear_bit());// disable i2c
    // select controller mode & speed
    i2c0.ic_con().modify(|_, w| {
        w.speed().fast();
        w.master_mode().enabled();
        w.ic_slave_disable().slave_disabled();
        w.ic_restart_en().enabled();
        w.tx_empty_ctrl().enabled()
    });
    // Clear FIFO threshold
    i2c0.ic_tx_tl().write(|w| unsafe { w.tx_tl().bits(0) });
    i2c0.ic_rx_tl().write(|w| unsafe { w.rx_tl().bits(0) });
                
    // IC_xCNT = (ROUNDUP(MIN_SCL_HIGH_LOWtime*OSCFREQ,0))
    // IC_HCNT = (600ns * 125MHz) + 1
    // IC_LCNT = (1300ns * 125MHz) + 1
    i2c0.ic_fs_scl_hcnt().write(|w| unsafe { w.ic_fs_scl_hcnt().bits(76) });
    i2c0.ic_fs_scl_lcnt().write(|w| unsafe { w.ic_fs_scl_lcnt().bits(163) });
    // spkln = lcnt/16;
    i2c0.ic_fs_spklen().write(|w| unsafe {  w.ic_fs_spklen().bits(163/16) });
    // sda_tx_hold_count = freq_in [cycles/s] * 300ns for scl < 1MHz
    let sda_tx_hold_count = ((125_000_000 * 3) / 10000000) + 1;
    i2c0.ic_sda_hold().modify(|_r, w| unsafe { w.ic_sda_tx_hold().bits(sda_tx_hold_count as u16) });

    // pwm2 set up
    let pwm2 = dp.PWM.ch(2);// Acquire handle for pwm slice 2
    resets.reset().modify(|_, w| w.pwm().clear_bit());// Deassert pwm

    // Configuring pwm2
    pwm2.csr().modify(|_, w| w
                            .divmode().div()// free runnning counter determined by fractional divider
                            .ph_correct().set_bit()// enable phase correction 
                            );
    pwm2.top().modify(|_, w| unsafe { w.bits(49_999) });// sets the wrap value: TOP + 1
    // For a fpwm of 50Hz w/ top of 50000 div need to be 25
    pwm2.div().modify(|_, w| unsafe { w.int().bits(25).frac().bits(0) });
    pwm2.csr().modify(|_, w| w.en().set_bit());// Enable pwm2

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

    // Configure gp4 as I2C0 SDA pin
    pads_bank0.gpio(4).modify(|_, w| w
                               .pue().set_bit() //pull up enable
                               .od().clear_bit() //output disable
                               .ie().set_bit() //input enable
                               );

    io_bank0.gpio(4).gpio_ctrl().modify(|_, w| w.funcsel().i2c()); 

    // Configure gp5 as I2C0 SCL pin
    pads_bank0.gpio(5).modify(|_, w| w
                               .pue().set_bit() //pull up enable
                               .od().clear_bit() //output disable
                               .ie().set_bit() //input enable
                               );

    io_bank0.gpio(5).gpio_ctrl().modify(|_, w| w.funcsel().i2c());

    // Configure GP20 as PWM channel 1A
    pads_bank0.gpio(20).modify(|_, w| w
                                .pue().set_bit()// pull up enable
                                .pde().set_bit()// pull down enable
                                .od().clear_bit()// output disable
                                .ie().set_bit()// input enable
                                );

    io_bank0.gpio(20).gpio_ctrl().modify(|_, w| w.funcsel().pwm());// connect to matching pwm

    // Configure GP16 for Relay 1 Enable
    pads_bank0.gpio(16).modify(|_, w| w
                               .pue().set_bit()// pullup enable
                               .pde().set_bit()// pulldown enable
                               .od().clear_bit()// output enable
                               .ie().set_bit()// input enable
                               );

    io_bank0.gpio(16).gpio_ctrl().modify(|_, w| w.funcsel().sio());// set function as sio

    sio.gpio_oe().modify(|r, w| unsafe { w.bits(r.gpio_oe().bits() | 1 << 16)});// Output enable for pin 16

    // Configure GP17 for Relay 2 Enable
    pads_bank0.gpio(17).modify(|_, w| w
                               .pue().set_bit()// pullup enable
                               .pde().set_bit()// pulldown enable
                               .od().clear_bit()// output enable
                               .ie().set_bit()// input enable
                               );

    io_bank0.gpio(17).gpio_ctrl().modify(|_, w| w.funcsel().sio());// set function as sio

    // Configure GP16 for Relay 1 Enable
    pads_bank0.gpio(16).modify(|_, w| w
                               .pue().set_bit()// pullup enable
                               .pde().set_bit()// pulldown enable
                               .od().clear_bit()// output enable
                               .ie().set_bit()// input enable
                               );

    io_bank0.gpio(16).gpio_ctrl().modify(|_, w| w.funcsel().sio());// set function as sio

    sio.gpio_oe().modify(|r, w| unsafe { w.bits(r.gpio_oe().bits() | 1 << 16)});// Output enable for pin 16

    // Configure GP17 for Relay 2 Enable
    pads_bank0.gpio(17).modify(|_, w| w
                               .pue().set_bit()// pullup enable
                               .pde().set_bit()// pulldown enable
                               .od().clear_bit()// output enable
                               .ie().set_bit()// input enable
                               );

    io_bank0.gpio(17).gpio_ctrl().modify(|_, w| w.funcsel().sio());// set function as sio

    sio.gpio_oe().modify(|r, w| unsafe { w.bits(r.gpio_oe().bits() | 1 << 17)});// Output enable for pin 17

    // Test 
    writeln!(serialbuf, "\nSelf-Aligning Satellite Dish in Rust\n").unwrap();
    transmit_uart_data(&uart_data, serialbuf);

    // Delay handle
    let mut delay = Delay::new(cp.SYST, 125_000_000);

    // Buffers
    let gpsbuf = String::new();// buffer to hold gps data
    let serialbuf = &mut String::<164>::new();// buffer to hold data to be serially transmitted

    configure_compass(&mut i2c0, &uart_data);

    // Variables
    let (mut theta, mut phi) = (0., 0.);// look angles
    let mut adjusted_theta: f32 = 0.;// adjusted theta; new theta to track, considers tilt
    let mut heading = 0.;// magnetic heading
    let mut ref_theta = 0.;// reference theta; initial value to be referenced for variance

    let mut position = Position::new();// gps position struct
    let mut tilt_angle = 90.; // initialize tilt angle i.e. kit facing up

    position_kit_tilt(pwm2, &mut tilt_angle);// start position of tilt

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
            let mut direction = DIRECTION.borrow(cs).borrow_mut();
            let mut uart_data = UARTDATA.borrow(cs).borrow_mut();
            let mut buffer = BUFFER.borrow(cs).borrow_mut();
            let mut sio = SIO.borrow(cs).borrow_mut();

            if AUTO.borrow(cs).get() {
                // Auto mode
                if GGA_ACQUIRED.borrow(cs).get() {

                    // Update the coordinates from the nmea sentence
                    parse_gga(buffer.as_mut().unwrap(), &mut position);

                    // Calculate look angles
                    (theta, phi) = get_look_angles(
                        position.lat,
                        position.long,
                        position.alt);

                    writeln!(serialbuf, "theta: {}  phi: {}", theta, phi).unwrap();
                    transmit_uart_data(
                        uart_data.as_mut().unwrap(),
                        serialbuf);

                    // Confirm theta == ref_theta from last iteration
                    // otherwise go back to first iteration
                    if theta.round() != ref_theta.round() {
                        AUTO_FIRST_ITER.borrow(cs).set(true);
                    }

                    if AUTO_FIRST_ITER.borrow(cs).get() {
                        // Tilt to face up for accurate heading readings
                        tilt_angle = 90.;
                        position_kit_tilt(pwm2, &mut tilt_angle);
                        
                        delay.delay_ms(250);// wait for tilt kit to arrive at 90 deg
                        
                        // Get the magnetic heading
                        heading = get_magnetic_heading(&mut i2c0);

                        writeln!(serialbuf, "heading: {} deg.\ntilt: {} deg.",
                                 heading.round(), &mut tilt_angle).unwrap();
                        transmit_uart_data(
                            uart_data.as_mut().unwrap(),
                            serialbuf);
                    } else {
                        heading = get_magnetic_heading(&mut i2c0);// get the adjusted magnetic heading i.e. at a tilt
                        theta = adjusted_theta;// we'll now track adjusted theta since we're at a tilt

                        writeln!(serialbuf, "adjusted theta: {} deg.\ntilt: {} deg.",
                                 heading.round(), &mut tilt_angle).unwrap();
                        transmit_uart_data(
                            uart_data.as_mut().unwrap(),
                            serialbuf);
                    }

                    // Pan moded servo cw while heading != theta i.e.
                    // Dish is not locked with theta
                    while !heading_within_range(theta, heading) {
                        if pan_clockwise(theta, heading) {
                            pan(&mut delay, sio.as_mut().unwrap(), Direction::Cw);

                            heading = get_magnetic_heading(&mut i2c0);
                        
                            writeln!(serialbuf, "Pan Cw: {} --> {}",
                                     heading.round(), theta.round()).unwrap();
                            transmit_uart_data(
                                uart_data.as_mut().unwrap(),
                                serialbuf);
                        } else {
                            pan(&mut delay, sio.as_mut().unwrap(), Direction::Ccw);

                            heading = get_magnetic_heading(&mut i2c0);

                            writeln!(serialbuf, "Pan Ccw: {} --> {}",
                                     heading.round(), theta.round()).unwrap();
                            transmit_uart_data(
                                uart_data.as_mut().unwrap(),
                                serialbuf);
                        }
                    }

                    // ...then tilt to lock with the azimuth, phi
                    while tilt_angle.round() != phi.round() {
                        if tilt_up(phi, tilt_angle) {
                            tilt(pwm2, &mut delay, &mut tilt_angle, Direction::Up);
                            
                            writeln!(serialbuf, "Tilt +: {} --> {}",
                                     &mut tilt_angle.round(), phi.round()).unwrap();
                            transmit_uart_data(
                                uart_data.as_mut().unwrap(),
                                serialbuf);
                        } else {
                            tilt(pwm2, &mut delay, &mut tilt_angle, Direction::Down);

                            writeln!(serialbuf, "Tilt -: {} --> {}", 
                                     &mut tilt_angle.round(), phi.round()).unwrap();
                            transmit_uart_data(
                                uart_data.as_mut().unwrap(),
                                serialbuf);
                        }
                    }

                    if AUTO_FIRST_ITER.borrow(cs).get() {
                        delay.delay_ms(100);// settle before getting adjusted_theta
                        
                        // Get the adjusted magnetic heading i.e.
                        // heading with system tilted
                        heading = get_magnetic_heading(&mut i2c0);
                        adjusted_theta = heading; 

                        // Keep theta for reference
                        ref_theta = theta;

                        AUTO_FIRST_ITER.borrow(cs).set(false);// clear flag
                    }

                    GGA_ACQUIRED.borrow(cs).set(false);// clear the flag
                }
            } else {
                // Manual mode
                match direction.as_mut() {
                    Some(Direction::Cw) => {
                        *direction = None;// reset direction
                        writeln!(serialbuf, "manual mode: cw").unwrap();
                        transmit_uart_data(uart_data.as_mut().unwrap(), serialbuf);

                        pan(&mut delay, sio.as_mut().unwrap(), Direction::Cw);
                        
                        heading = get_magnetic_heading(&mut i2c0);
                        writeln!(serialbuf, "Heading: {}", heading.round()).unwrap();
                        transmit_uart_data(
                            uart_data.as_mut().unwrap(),
                            serialbuf);
                    },
                    Some(Direction::Ccw) => {
                        *direction = None;
                        writeln!(serialbuf, "manual mode: ccw").unwrap();
                        transmit_uart_data(uart_data.as_mut().unwrap(), serialbuf);

                        pan(&mut delay, sio.as_mut().unwrap(), Direction::Ccw);

                        heading = get_magnetic_heading(&mut i2c0);
                        writeln!(serialbuf, "Heading: {}", heading.round()).unwrap();
                        transmit_uart_data(
                            uart_data.as_mut().unwrap(),
                            serialbuf);
                    },
                    Some(Direction::Up) => {
                        *direction = None;
                        writeln!(serialbuf, "manual mode: up").unwrap();
                        transmit_uart_data(uart_data.as_mut().unwrap(), serialbuf);

                        tilt(pwm2, &mut delay, &mut tilt_angle, Direction::Up);

                        writeln!(serialbuf, "tilt angle: {}", tilt_angle.round()).unwrap();
                        transmit_uart_data(uart_data.as_mut().unwrap(), serialbuf);
                    },
                    Some(Direction::Down) => {
                        *direction = None;
                        writeln!(serialbuf, "manual mode: down").unwrap();
                        transmit_uart_data(uart_data.as_mut().unwrap(), serialbuf);

                        tilt(pwm2, &mut delay, &mut tilt_angle, Direction::Down);

                        writeln!(serialbuf, "tilt angle: {}", tilt_angle.round()).unwrap();
                        transmit_uart_data(uart_data.as_mut().unwrap(), serialbuf);
                    },
                    Some(Direction::Zero) => {
                        *direction = None;
                        writeln!(serialbuf, "manual mode: position zero").unwrap();
                        transmit_uart_data(uart_data.as_mut().unwrap(), serialbuf);

                        // Face 0° and look up
                        tilt_angle = 90.;
                        position_kit_tilt(pwm2, &mut tilt_angle);
                        
                        delay.delay_ms(250);// wait for tilt kit to arrive at 90 deg

                        heading = get_magnetic_heading(&mut i2c0);

                        theta = 0.;

                        while !heading_within_range(theta, heading) {
                            if pan_clockwise(theta, heading) {
                                pan(&mut delay, sio.as_mut().unwrap(), Direction::Cw);
                            
                                heading = get_magnetic_heading(&mut i2c0);
                            } else {
                                pan(&mut delay, sio.as_mut().unwrap(), Direction::Ccw);
                            
                                heading = get_magnetic_heading(&mut i2c0);
                            }
                        }
                            
                        writeln!(serialbuf, "heading: {}   tilt angle: {}", 
                                 heading.round(), tilt_angle).unwrap();
                        transmit_uart_data(uart_data.as_mut().unwrap(), serialbuf);
                    },
                    None => {},
                }
            }
        });

        cortex_m::asm::wfi();// wait for interrupt
    }
}

fn tilt_up(phi: f32, tilt_angle: f32) -> bool {
    let tilt = phi - tilt_angle;

    tilt < 0.
}

fn heading_within_range(theta: f32, heading: f32) -> bool {
    // checks whether heading is +/- 2 of desired pan angle
    heading.round() == theta.round() || heading.round() == theta.round() - 1. 
        || heading.round() == theta.round() - 2.
        || heading.round() == theta.round() + 1.
        || heading.round() == theta.round() + 2.
}

fn pan_clockwise(theta: f32, heading: f32) -> bool {
    // Check if pan sd be cw or ccw 
    let pan = theta - heading;

    if pan.abs() < 180. {
        pan > 0.
    } else {
        pan < 0.
    }
}

fn pan(delay: &mut Delay, sio: &mut SIO, direction: Direction) {
    match direction {
        Direction::Cw => {
            // Move servo CW: Relay 1 ON, Relay 2 OFF
            sio.gpio_out().modify(|r, w| unsafe { w.bits(r.gpio_out().bits() | 1 << 16)});// Set pin 16 high

            delay.delay_ms(10);// delay

            // Put servo OFF
            sio.gpio_out().modify(|r, w| unsafe { w.bits(r.gpio_out().bits() & !(1 << 16))});// Set pin 16 low
            sio.gpio_out().modify(|r, w| unsafe { w.bits(r.gpio_out().bits() & !(1 << 17))});// Set pin 17 low
        },
        Direction::Ccw => {
            // Move servo CCW: Relay 2 ON, Relay 1 OFF
            sio.gpio_out().modify(|r, w| unsafe { w.bits(r.gpio_out().bits() | 1 << 17)});// Set pin 17 high

            delay.delay_ms(10);// delay

            // Put servo OFF
            sio.gpio_out().modify(|r, w| unsafe { w.bits(r.gpio_out().bits() & !(1 << 16))});// Set pin 16 low
            sio.gpio_out().modify(|r, w| unsafe { w.bits(r.gpio_out().bits() & !(1 << 17))});// Set pin 17 low
        },
        _ => {},
    }
}

fn tilt(pwm: &CH, delay: &mut Delay, tilt_angle: &mut f32, direction: Direction) {
    if *tilt_angle >= 0. && *tilt_angle <= 90. { 
        // valid movts only if angle within range
        match direction {
            Direction::Up => {
                if *tilt_angle != 0. {// constrain angle within range
                    *tilt_angle -= 1.;// update tilt angle
                    position_kit_tilt(pwm, tilt_angle);
                    delay.delay_ms(1);// delay
                }
            },
            Direction::Down => {
                if *tilt_angle != 90. {// constrain angle within range
                    *tilt_angle += 1.;// update tilt angle
                    position_kit_tilt(pwm, tilt_angle);
                    delay.delay_ms(1);// delay
                }
            },
            _ => {},
        }
    }
}

fn position_kit_tilt(pwm: &CH, tilt_angle: &mut f32) {
    // facing up --> at 90 deg
    // standing vertically --> 0 deg 
    sweep(pwm, (90. - tilt_angle.round()) as u16);
}

pub fn sweep(pwm: &CH, mut degrees: u16) {
    const BASE: u16 = 1400;// here pwm at position 0 degrees

    degrees = BASE + (degrees * 28);// ~28: movt by 1 degree 
    unsafe {
        pwm.cc().modify(|_, w| w.a().bits(degrees) );// move by one degree * no of degrees
    }
}

fn receive_uart_cmd(uart: &UART1) -> u8 {// receive 1 byte
    while uart.uartfr().read().rxfe().bit_is_set() {} // wait until byte received

    uart.uartdr().read().data().bits()// Received data
}

fn _transmit_uart_cmd(uart: &UART1, buffer: &mut String<164>) {
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

fn i2c_read(
    i2c0: &mut I2C0,
    addr: u16,
    bytes: &mut [u8],
) -> Result<(), u32> {
    i2c0.ic_enable().modify(|_, w| w.enable().clear_bit());// disable i2c
    i2c0.ic_tar().modify(|_, w| unsafe { w.ic_tar().bits(addr) });// slave address
    i2c0.ic_enable().modify(|_, w| w.enable().set_bit());// enable i2c

    let last_index = bytes.len() - 1;
    for (i, byte) in bytes.iter_mut().enumerate() {
        let first = i == 0;
        let last = i == last_index;

        // wait until there is space in the FIFO to write the next byte
        while TX_FIFO_SIZE - i2c0.ic_txflr().read().txflr().bits() == 0 {}

        i2c0.ic_data_cmd().modify(|_, w| {
            if first {
                w.restart().enable();
            } else {
                w.restart().disable();
            }

            if last {
                w.stop().enable();
            } else {
                w.stop().disable();
            }

            w.cmd().read()
        }); 

        //Wait until address tx'ed
        while i2c0.ic_raw_intr_stat().read().tx_empty().is_inactive() {}
        //Clear ABORT interrupt
        //self.i2c0.ic_clr_tx_abrt.read();

        while i2c0.ic_rxflr().read().bits() == 0 {
            //Wait while receive FIFO empty
            //If attempt aborts : not valid address; return error with
            //abort reason.
            let abort_reason = i2c0.ic_tx_abrt_source().read().bits();
            //Clear ABORT interrupt
            i2c0.ic_clr_tx_abrt().read();
            if abort_reason != 0 {
                return Err(abort_reason)
            } 
        }

        *byte = i2c0.ic_data_cmd().read().dat().bits();
    }

    Ok(())
}

fn i2c_write(
    i2c0: &mut I2C0,
    addr: u16,
    bytes: &[u8],
) -> Result<(), u32> {
    i2c0.ic_enable().modify(|_, w| w.enable().clear_bit());// disable i2c
    i2c0.ic_tar().modify(|_, w| unsafe { w.ic_tar().bits(addr) });// slave address
    i2c0.ic_enable().modify(|_, w| w.enable().set_bit());// enable i2c

    let last_index = bytes.len() - 1;
    for (i, byte) in bytes.iter().enumerate() {
        let last = i == last_index;

        i2c0.ic_data_cmd().modify(|_, w| {
            if last {
                w.stop().enable();
            } else {
                w.stop().disable();
            }
            unsafe { w.dat().bits(*byte)}
        }); 

        // Wait until address and data tx'ed
        while i2c0.ic_raw_intr_stat().read().tx_empty().is_inactive() {}
        // Clear ABORT interrupt
        // self.i2c0.ic_clr_tx_abrt.read();

        // If attempt aborts : not valid address; return error with
        // abort reason.
        let abort_reason = i2c0.ic_tx_abrt_source().read().bits();
        // Clear ABORT interrupt
        i2c0.ic_clr_tx_abrt().read();
        if abort_reason != 0 {
            // Wait until the STOP condition has occured
            while i2c0.ic_raw_intr_stat().read().stop_det().is_inactive() {}
            // Clear STOP interrupt
            i2c0.ic_clr_stop_det().read().clr_stop_det();
            return Err(abort_reason)
        } 

        if last {
            // Wait until the STOP condition has occured
            while i2c0.ic_raw_intr_stat().read().stop_det().is_inactive() {}
            // Clear STOP interrupt
            i2c0.ic_clr_stop_det().read().clr_stop_det();
        }
    }

    Ok(())
}

fn configure_compass(i2c0: &mut I2C0, uart: &UART0) { // Configure and confirm HMC5833L
    let mut writebuf: [u8; 1];// buffer to hold 1 byte
    let mut writebuf2: [u8; 2];// buffer to hold 2 bytes
    let mut readbuf: [u8; 1] = [0; 1];
    let serialbuf = &mut String::<164>::new();

    writeln!(serialbuf, "Compass configuration.").unwrap();
    transmit_uart_data(uart, serialbuf);

    // ID the compass
    // Read ID REG A
    writebuf = [IDENTIFICATION_REG_A];
    i2c_write(i2c0, HMC5883L_ADDR, &writebuf).unwrap();
    i2c_read(i2c0, HMC5883L_ADDR, &mut readbuf).unwrap();
    let id_a = readbuf[0];
    writeln!(serialbuf, "Id reg a: 0x{:02X}", id_a).unwrap();
    transmit_uart_data(uart, serialbuf);

    // Read ID REG B 
    writebuf = [IDENTIFICATION_REG_B];
    i2c_write(i2c0, HMC5883L_ADDR, &writebuf).unwrap();
    i2c_read(i2c0, HMC5883L_ADDR, &mut readbuf).unwrap();
    let id_b = readbuf[0];
    writeln!(serialbuf, "Id reg b: 0x{:02X}", id_b).unwrap();
    transmit_uart_data(uart, serialbuf);

    // Read ID REG C
    writebuf = [IDENTIFICATION_REG_C];
    i2c_write(i2c0, HMC5883L_ADDR, &writebuf).unwrap();
    i2c_read(i2c0, HMC5883L_ADDR, &mut readbuf).unwrap();
    let id_c = readbuf[0];
    writeln!(serialbuf, "Id reg c: 0x{:02X}", id_c).unwrap();
    transmit_uart_data(uart, serialbuf);

    if id_a == 0x48 && id_b == 0x34 && id_c == 0x33 {
        writeln!(serialbuf, "Magnetometer ID confirmed!").unwrap();
        transmit_uart_data(uart, serialbuf);
    }

    // Set compass in continuous mode & confirm
    writebuf2 = [MODE_R, 0x0];
    i2c_write(i2c0, HMC5883L_ADDR, &writebuf2).unwrap();

    writebuf = [MODE_R];
    i2c_write(i2c0, HMC5883L_ADDR, &writebuf).unwrap();
    i2c_read(i2c0, HMC5883L_ADDR, &mut readbuf).unwrap();
        
    let mode = readbuf[0];
    writeln!(serialbuf, "Mode reg: 0b{:08b}", mode).unwrap();
    transmit_uart_data(uart, serialbuf);

    //let mode = readbuf[0];
    if (mode & 1 << 1) == 0 && (mode & 1 << 0) == 0  { 
        writeln!(serialbuf, "continuous mode set.").unwrap();
        transmit_uart_data(uart, serialbuf);
    } else if (mode & 1 << 1) == 0 && (mode & 1 << 0) != 0 {
        writeln!(serialbuf, "single-measurement mode set.").unwrap();
        transmit_uart_data(uart, serialbuf);
    } else {
        writeln!(serialbuf, "device in idle mode.").unwrap();
        transmit_uart_data(uart, serialbuf);
    }

    // Set data output rate & number of samples & confirm
    // sample avg = 8; data output rate = 15Hz; normal measurement cfgn
    writebuf2 = [CFG_REG_A, 0b01110000];
    i2c_write(i2c0, HMC5883L_ADDR, &writebuf2).unwrap();
    writebuf = [CFG_REG_A];
    i2c_write(i2c0, HMC5883L_ADDR, &writebuf).unwrap();
    i2c_read(i2c0, HMC5883L_ADDR, &mut readbuf).unwrap();

    let cfg_a = readbuf[0];
    writeln!(serialbuf, "cfg reg a: 0b{:08b}", cfg_a).unwrap();
    transmit_uart_data(uart, serialbuf);

    // Set Gain & confirm
    // gain = 1090 LSB/Gauss
    writebuf2 = [CFG_REG_B, 0b00100000];
    i2c_write(i2c0, HMC5883L_ADDR, &writebuf2).unwrap();

    writebuf = [CFG_REG_B];
    i2c_write(i2c0, HMC5883L_ADDR, &writebuf).unwrap();
    i2c_read(i2c0, HMC5883L_ADDR, &mut readbuf).unwrap();

    let cfg_b = readbuf[0];
    writeln!(serialbuf, "cfg reg b: 0b{:08b}", cfg_b).unwrap();
    transmit_uart_data(uart, serialbuf);

    if (cfg_a == 0b01110000) && (cfg_b == 0b00100000)   { 
        writeln!(serialbuf, "cfg regs set.").unwrap();
        transmit_uart_data(uart, serialbuf);
    }
}

fn get_magnetic_heading(i2c0: &mut I2C0) -> f32 { 
    // Read raw data from compass.
    // Capture 50 heading samples and return the average
    let mut count = 0.;
    let samples = 50.;
    let mut headings = 0.;
    
    while count != samples {
        // Point to the address of DATA_OUTPUT_X_MSB_R
        let writebuf = [DATA_OUTPUT_X_MSB_R];
        i2c_write(i2c0, HMC5883L_ADDR, &writebuf).unwrap();
        // Read the output of the HMC5883L
        // All six registers are read into the
        // rawbuf buffer
        let mut rawbuf: [u8; 6] = [0; 6];
        i2c_read(i2c0, HMC5883L_ADDR, &mut rawbuf).unwrap();

        let x_h = rawbuf[0] as u16;
        let x_l = rawbuf[1] as u16;
        let z_h = rawbuf[2] as u16;
        let z_l = rawbuf[3] as u16;
        let y_h = rawbuf[4] as u16;
        let y_l = rawbuf[5] as u16;

        let x = ((x_h << 8) + x_l) as i16;
        let y = ((y_h << 8) + y_l) as i16;
        let _z = ((z_h << 8) + z_l) as i16;

        // North-Clockwise convention for getting the heading
        let mut heading = (y as f32 - 75.).atan2(x as f32 + 75.);

        heading += MAGNETIC_DECLINATION*(PI/180.);// add declination in radians

        // Check for sign
        if heading < 0. {
            heading += 2.*PI;
        }
        
        // Check for value wrap
        if heading > 2.*PI {
            heading -= 2.*PI;
        }

        heading *= 180./PI;

        headings += heading;
        
        count += 1.;
    }

    headings/samples
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

        uart.as_mut().unwrap().uarticr().modify(|_, w| w.rtic().bit(true));// clear rx interrupt

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
