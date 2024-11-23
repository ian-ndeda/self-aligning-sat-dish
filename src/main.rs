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
    Interrupt, interrupt,
};

mod drivers;
use crate::drivers::{
    i2c::i2c0::I2c0,
    timer::Timer,
    pwm::{Pwm, PwmSlices},
    dma::{Dma, DmaChannels},
    uart::{uart0::Uart0, uart1::Uart1}, 
    clocks::{resets::Resets, xosc::Xosc, clockz::Clocks, watchdog::Watchdog},
    gpio::{gpios::{Pin, Gpio, typestate::{Output, FunctionSio, PushPull}}}
};

// Global constants
const UART0_DR: u32 = 0x40034000;

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

// Types aliases
type Pin25 = Pin<Output<FunctionSio<PushPull>>, 25>;
type Relay1 = Pin<Output<FunctionSio<PushPull>>, 16>;
type Relay2 = Pin<Output<FunctionSio<PushPull>>, 17>;

// Global variable for peripherals
static TIMER: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));
static GP25: Mutex<RefCell<Option<Pin25>>> = Mutex::new(RefCell::new(None));
static UARTCMD: Mutex<RefCell<Option<Uart1>>> = Mutex::new(RefCell::new(None));
static UARTDATA: Mutex<RefCell<Option<Uart0>>> = Mutex::new(RefCell::new(None));

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
    let xosc = Xosc::new(dp.XOSC);// handle for xosc
    xosc.init_xosc();// initialize xosc
    
    // Clocks configuration
    let resets = Resets::new(dp.RESETS);
    let clocks = Clocks::new(dp.CLOCKS, dp.PLL_SYS); 
    clocks.configure(&resets);

    // Watchdog setup for timer operation
    let watchdog = Watchdog::new(dp.WATCHDOG);
    watchdog.start_ticks();

    // Timer setup
    let timer = Timer::new(dp.TIMER, &resets);
    timer.enable_alarm0_interrupt();
    timer.set_alarm0_ms(1_000);

    // Configure pin 25 as output: LED
    let gpio = Gpio::new(dp.IO_BANK0, dp.PADS_BANK0, &resets).split();// Enable all gpios
    let gp25 = gpio.gp25.as_output().as_sio().as_pushpull(); 

    // Configure uart1 pins
    let tx1 = gpio.gp8.as_output().as_uart().as_pushpull();// connected to BT05 rx pin
    let rx1 = gpio.gp9.as_input().as_uart().as_pushpull();// connected to BT05 tx pin

    // Configure uart1: command reception
    let uart_cmd = Uart1::new(dp.UART1, tx1, rx1, &resets);
    uart_cmd.configure_9600();
    uart_cmd.enable_uart1();
    uart_cmd.enable_interrupt();
    uart_cmd.enable_tx_dma();

    // Configure uart0 pins
    let tx0 = gpio.gp0.as_output().as_uart().as_pushpull();
    let rx0 = gpio.gp1.as_input().as_uart().as_pushpull();// connected to gps tx pin

    // Configure uart0: gps reception
    let uart_data = Uart0::new(dp.UART0, tx0, rx0, &resets);
    uart_data.configure_9600();
    uart_data.enable_uart0();
    uart_data.enable_interrupt();
    uart_data.enable_tx_dma();

    // Configure i2c0 pins
    let sda = gpio.gp4.as_bidirectional().as_i2c().as_pull_up();// connected to compass sda
    let scl = gpio.gp5.as_bidirectional().as_i2c().as_pull_up();// connected to compass scl

    // I2c setup
    let i2c = I2c0::new(dp.I2C0, sda, scl, &resets);
    i2c.configure_as_master();

    // Configure control pins for servo relays
    let relay1 = gpio.gp16.as_output().as_sio().as_pushpull();
    let relay2 = gpio.gp17.as_output().as_sio().as_pushpull();

    // Configure gp20 as pwm 2A output
    let pwm_2_a = gpio.gp20.as_output().as_pwm().as_pushpull();

    // Pwm2 setup
    let pwm_slices = PwmSlices::new(dp.PWM, &resets);
    let pwm2 = pwm_slices.pwm2;
    pwm2.configure_50_hz(pwm_2_a).enable();

    // Configure dma ch0 as conduit between
    // uart0 dr and serialbuf location
    let dma_channels = DmaChannels::new(dp.DMA, &resets);
    let dma0 = dma_channels.dma0.configure();
    dma0.set_write_addr(UART0_DR);

    // Delay handle
    let mut delay = Delay::new(cp.SYST, 125_000_000);
    
    // Buffers
    let gpsbuf = String::<164>::new();// buffer to hold gps data
    let serialbuf = &mut String::<164>::new();// buffer to hold data to be serially transmitted

    // Test 
    writeln!(serialbuf, "\nSelf-Aligning Satellite Dish in Rust\n").unwrap();
    dma0.transfer(serialbuf);

    configure_compass(&i2c, &dma0);

    // Variables
    let (mut theta, mut phi) = (0., 0.);// look angles
    let mut adjusted_theta: f32 = 0.;// adjusted theta; new theta to track, considers tilt
    let mut heading = 0.;// magnetic heading
    let mut ref_theta = 0.;// reference theta; initial value to be referenced for variance

    let mut position = Position::new();// gps position struct
    let mut tilt_angle = 90.; // initialize tilt angle i.e. kit facing up
    
    position_kit_tilt(&pwm2, &mut tilt_angle);// start position of tilt

    gp25.set_pin();// put indication LED on

    // Move peripherals into global scope
    cortex_m::interrupt::free(|cs| {
        GP25.borrow(cs).replace(Some(gp25));
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
            let mut buffer = BUFFER.borrow(cs).borrow_mut();

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

                    writeln!(serialbuf, "\ntheta: {}  phi: {}", theta, phi).unwrap();
                    dma0.transfer(serialbuf);

                    // Confirm theta == ref_theta from last iteration
                    // otherwise go back to first iteration
                    if theta.round() != ref_theta.round() {
                        AUTO_FIRST_ITER.borrow(cs).set(true);
                    }

                    if AUTO_FIRST_ITER.borrow(cs).get() {
                        // Tilt to face up for accurate heading readings
                        tilt_angle = 90.;
                        position_kit_tilt(&pwm2, &mut tilt_angle);
                        
                        delay.delay_ms(250);// wait for tilt kit to arrive at 90 deg
                        
                        // Get the magnetic heading
                        heading = get_magnetic_heading(&i2c);

                        writeln!(serialbuf, "> heading: {} deg.\n> tilt: {} deg.",
                                 heading.round(), &mut tilt_angle).unwrap();
                        dma0.transfer(serialbuf);
                    } else {
                        let tilted_heading = get_magnetic_heading(&i2c);// get the adjusted magnetic heading i.e. at a tilt
                        heading = tilted_heading;
                        theta = adjusted_theta;// we'll now track adjusted theta since we're at a tilt

                        writeln!(serialbuf, "adjusted theta: {} deg.", theta).unwrap();
                        dma0.transfer(serialbuf);

                        writeln!(serialbuf, "> tilted heading: {} deg.\n> tilt: {} deg.",
                                 heading.round(), &mut tilt_angle).unwrap();
                        dma0.transfer(serialbuf);
                    }

                    // Pan moded servo cw while heading != theta i.e.
                    // Dish is not locked with theta
                    while !heading_within_range(theta, heading) {
                        if pan_clockwise(theta, heading) {
                            pan(&mut delay, (&relay1, &relay2), Direction::Cw);

                            heading = get_magnetic_heading(&i2c);
                        
                            writeln!(serialbuf, "Pan Cw: {} --> {}",
                                     heading.round(), theta.round()).unwrap();
                            dma0.transfer(serialbuf);
                        } else {
                            pan(&mut delay, (&relay1, &relay2), Direction::Ccw);

                            heading = get_magnetic_heading(&i2c);

                            writeln!(serialbuf, "Pan Ccw: {} --> {}",
                                     heading.round(), theta.round()).unwrap();
                            dma0.transfer(serialbuf);
                        }
                    }

                    // ...then tilt to lock with the azimuth, phi
                    while tilt_angle.round() != phi.round() {
                        if tilt_up(phi, tilt_angle) {
                            tilt(&pwm2, &mut delay, &mut tilt_angle, Direction::Up);
                            
                            writeln!(serialbuf, "Tilt +: {} --> {}",
                                     &mut tilt_angle.round(), phi.round()).unwrap();
                            dma0.transfer(serialbuf);
                        } else {
                            tilt(&pwm2, &mut delay, &mut tilt_angle, Direction::Down);

                            writeln!(serialbuf, "Tilt -: {} --> {}", 
                                     &mut tilt_angle.round(), phi.round()).unwrap();
                            dma0.transfer(serialbuf);
                        }
                    }

                    if AUTO_FIRST_ITER.borrow(cs).get() {
                        delay.delay_ms(100);// settle before getting adjusted_theta
                        
                        // Get the adjusted magnetic heading i.e.
                        // heading with system tilted
                        heading = get_magnetic_heading(&i2c);
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
                        dma0.transfer(serialbuf);

                        pan(&mut delay, (&relay1, &relay2), Direction::Cw);
                        
                        heading = get_magnetic_heading(&i2c);
                        writeln!(serialbuf, "Heading: {}", heading.round()).unwrap();
                        dma0.transfer(serialbuf);
                    },
                    Some(Direction::Ccw) => {
                        *direction = None;
                        writeln!(serialbuf, "manual mode: ccw").unwrap();
                        dma0.transfer(serialbuf);

                        pan(&mut delay, (&relay1, &relay2), Direction::Ccw);

                        heading = get_magnetic_heading(&i2c);
                        writeln!(serialbuf, "Heading: {}", heading.round()).unwrap();
                        dma0.transfer(serialbuf);
                    },
                    Some(Direction::Up) => {
                        *direction = None;
                        writeln!(serialbuf, "manual mode: up").unwrap();
                        dma0.transfer(serialbuf);

                        tilt(&pwm2, &mut delay, &mut tilt_angle, Direction::Up);

                        writeln!(serialbuf, "tilt angle: {}", tilt_angle.round()).unwrap();
                        dma0.transfer(serialbuf);
                    },
                    Some(Direction::Down) => {
                        *direction = None;
                        writeln!(serialbuf, "manual mode: down").unwrap();
                        dma0.transfer(serialbuf);

                        tilt(&pwm2, &mut delay, &mut tilt_angle, Direction::Down);

                        writeln!(serialbuf, "tilt angle: {}", tilt_angle.round()).unwrap();
                        dma0.transfer(serialbuf);
                    },
                    Some(Direction::Zero) => {
                        *direction = None;
                        writeln!(serialbuf, "manual mode: position zero").unwrap();
                        dma0.transfer(serialbuf);

                        // Face 0° and look up
                        tilt_angle = 90.;
                        position_kit_tilt(&pwm2, &mut tilt_angle);
                        
                        delay.delay_ms(250);// wait for tilt kit to arrive at 90 deg

                        heading = get_magnetic_heading(&i2c);

                        theta = 0.;

                        while !heading_within_range(theta, heading) {
                            if pan_clockwise(theta, heading) {
                                pan(&mut delay, (&relay1, &relay2), Direction::Cw);
                            
                                heading = get_magnetic_heading(&i2c);
                            } else {
                                pan(&mut delay, (&relay1, &relay2), Direction::Ccw);
                            
                                heading = get_magnetic_heading(&i2c);
                            }
                        }
                            
                        writeln!(serialbuf, "heading: {}   tilt angle: {}", 
                                 heading.round(), tilt_angle).unwrap();
                        dma0.transfer(serialbuf);
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
    // checks whether heading is +/- 3 of desired pan angle
    heading.round() == theta.round() || heading.round() == theta.round() - 1. 
        || heading.round() == theta.round() - 2.
        || heading.round() == theta.round() - 3.
        || heading.round() == theta.round() + 1.
        || heading.round() == theta.round() + 2.
        || heading.round() == theta.round() + 3.
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

fn pan(delay: &mut Delay, relays: (&Relay1, &Relay2), direction: Direction) {
    match direction {
        Direction::Cw => {
            // Move servo CW: Relay 1 ON, Relay 2 OFF
            relays.0.set_pin();

            delay.delay_ms(4);// delay

            // Put servo OFF
            relays.0.clear_pin();
            relays.1.clear_pin();
        },
        Direction::Ccw => {
            // Move servo CCW: Relay 2 ON, Relay 1 OFF
            relays.1.set_pin();

            delay.delay_ms(4);// delay

            // Put servo OFF
            relays.0.clear_pin();
            relays.1.clear_pin();
        },
        _ => {},
    }
}

fn tilt(pwm: &Pwm<2>, delay: &mut Delay, tilt_angle: &mut f32, direction: Direction) {
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

fn position_kit_tilt(pwm: &Pwm<2>, tilt_angle: &mut f32) {
    // facing up --> at 90 deg
    // standing vertically --> 0 deg 
    pwm.sweep((90. - tilt_angle.round()) as u16);
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

fn configure_compass(i2c: &I2c0, dma: &Dma<0>) { // Configure and confirm HMC5833L
    let mut writebuf: [u8; 1];// buffer to hold 1 byte
    let mut writebuf2: [u8; 2];// buffer to hold 2 bytes
    let mut readbuf: [u8; 1] = [0; 1];
    let serialbuf = &mut String::<164>::new();

    writeln!(serialbuf, "Compass configuration.").unwrap();
    dma.transfer(serialbuf);

    // ID the compass
    // Read ID REG A
    writebuf = [IDENTIFICATION_REG_A];
    i2c.write(HMC5883L_ADDR, &writebuf).unwrap();
    i2c.read(HMC5883L_ADDR, &mut readbuf).unwrap();
    let id_a = readbuf[0];
    writeln!(serialbuf, "Id reg a: 0x{:02X}", id_a).unwrap();
    dma.transfer(serialbuf);

    // Read ID REG B 
    writebuf = [IDENTIFICATION_REG_B];
    i2c.write(HMC5883L_ADDR, &writebuf).unwrap();
    i2c.read(HMC5883L_ADDR, &mut readbuf).unwrap();
    let id_b = readbuf[0];
    writeln!(serialbuf, "Id reg b: 0x{:02X}", id_b).unwrap();
    dma.transfer(serialbuf);

    // Read ID REG C
    writebuf = [IDENTIFICATION_REG_C];
    i2c.write(HMC5883L_ADDR, &writebuf).unwrap();
    i2c.read(HMC5883L_ADDR, &mut readbuf).unwrap();
    let id_c = readbuf[0];
    writeln!(serialbuf, "Id reg c: 0x{:02X}", id_c).unwrap();
    dma.transfer(serialbuf);

    if id_a == 0x48 && id_b == 0x34 && id_c == 0x33 {
        writeln!(serialbuf, "Magnetometer ID confirmed!").unwrap();
        dma.transfer(serialbuf);
    }

    // Set compass in continuous mode & confirm
    writebuf2 = [MODE_R, 0x0];
    i2c.write(HMC5883L_ADDR, &writebuf2).unwrap();

    writebuf = [MODE_R];
    i2c.write(HMC5883L_ADDR, &writebuf).unwrap();
    i2c.read(HMC5883L_ADDR, &mut readbuf).unwrap();
        
    let mode = readbuf[0];
    writeln!(serialbuf, "Mode reg: 0b{:08b}", mode).unwrap();
    dma.transfer(serialbuf);

    //let mode = readbuf[0];
    if (mode & 1 << 1) == 0 && (mode & 1 << 0) == 0  { 
        writeln!(serialbuf, "continuous mode set.").unwrap();
        dma.transfer(serialbuf);
    } else if (mode & 1 << 1) == 0 && (mode & 1 << 0) != 0 {
        writeln!(serialbuf, "single-measurement mode set.").unwrap();
        dma.transfer(serialbuf);
    } else {
        writeln!(serialbuf, "device in idle mode.").unwrap();
        dma.transfer(serialbuf);
    }

    // Set data output rate & number of samples & confirm
    // sample avg = 8; data output rate = 15Hz; normal measurement cfgn
    writebuf2 = [CFG_REG_A, 0b01110000];
    i2c.write(HMC5883L_ADDR, &writebuf2).unwrap();

    writebuf = [CFG_REG_A];
    i2c.write(HMC5883L_ADDR, &writebuf).unwrap();
    i2c.read(HMC5883L_ADDR, &mut readbuf).unwrap();

    let cfg_a = readbuf[0];
    writeln!(serialbuf, "cfg reg a: 0b{:08b}", cfg_a).unwrap();
    dma.transfer(serialbuf);

    // Set Gain & confirm
    // gain = 1090 LSB/Gauss
    writebuf2 = [CFG_REG_B, 0b00100000];
    i2c.write(HMC5883L_ADDR, &writebuf2).unwrap();

    writebuf = [CFG_REG_B];
    i2c.write(HMC5883L_ADDR, &writebuf).unwrap();
    i2c.read(HMC5883L_ADDR, &mut readbuf).unwrap();

    let cfg_b = readbuf[0];
    writeln!(serialbuf, "cfg reg b: 0b{:08b}", cfg_b).unwrap();
    dma.transfer(serialbuf);

    if (cfg_a == 0b01110000) && (cfg_b == 0b00100000)   { 
        writeln!(serialbuf, "cfg regs set.\n").unwrap();
        dma.transfer(serialbuf);
    }
}

fn get_magnetic_heading(i2c: &I2c0) -> f32 {
    // Read raw data from compass.
    // Capture 50 heading samples and return the average
    let mut count = 0.;
    let samples = 50.;
    let mut headings = 0.;
    
    while count != samples {
        // Point to the address of DATA_OUTPUT_X_MSB_R
        let writebuf = [DATA_OUTPUT_X_MSB_R];
        i2c.write(HMC5883L_ADDR, &writebuf).unwrap();
        // Read the output of the HMC5883L
        // All six registers are read into the
        // rawbuf buffer
        let mut rawbuf: [u8; 6] = [0; 6];
        i2c.read(HMC5883L_ADDR, &mut rawbuf).unwrap();

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
        let mut heading = (y as f32 - 50.).atan2(x as f32 + 20.);

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
    // Blink Led every 1 sec 
    cortex_m::interrupt::free(|cs| {
        let mut timer = TIMER.borrow(cs).borrow_mut();
        let mut gp25 = GP25.borrow(cs).borrow_mut();

        timer.as_mut().unwrap().clear_alarm0_interrupt();// clear interrupt
        gp25.as_mut().unwrap().toggle_pin();   
        timer.as_mut().unwrap().set_alarm0_ms(1_000);
    });
}

#[interrupt]
fn UART1_IRQ() {
    // Enter critical section
    cortex_m::interrupt::free(|cs| {
        // peripheral handles
        let mut uart = UARTCMD.borrow(cs).borrow_mut();
        let mut dir = DIRECTION.borrow(cs).borrow_mut();

        uart.as_mut().unwrap().clear_interrupt();

        // receive commands
        let b = uart.as_mut().unwrap().receive_byte();

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

        uart.as_mut().unwrap().clear_interrupt();

        // Data acquisition
        if started.get() {// If nmea sentence acquisition started
            if gga_confirmed.get() {
                let b = uart.as_mut().unwrap().receive_byte();// Received data
                buffer.as_mut().unwrap().push(char::from(b)).unwrap();// push to buffer
                if b == b'\n' {// End of nmea sentence
                    gga_acquired.set(true);// set flag
                    gga_confirmed.set(false);// unset confirmed flag
                    started.set(false);// unset flag; start again
                }
            } else {
                let b = uart.as_mut().unwrap().receive_byte();// Received data
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
            let b = uart.as_mut().unwrap().receive_byte();// Received data
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
