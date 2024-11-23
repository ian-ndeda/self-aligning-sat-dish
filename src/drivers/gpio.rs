use rp2040_pac::{SIO, IO_BANK0, PADS_BANK0};

use crate::drivers::clocks::resets::Resets;
    
pub mod gpios {
    use super::{SIO, IO_BANK0, PADS_BANK0, Resets};
    use cortex_m;
    use core::marker::PhantomData;


    pub trait PinDirection {}
    pub trait PinFunction {}
    pub trait PinMode {}

    pub mod typestate {
        use super::{PinFunction, PinMode, PhantomData};

        // Pin Direction states
        pub struct NotConfigured<F: PinFunction> {
             _marker: PhantomData<F>,
        }

        pub struct Input<F: PinFunction> {
             _marker: PhantomData<F>,
        }

        pub struct Output<F: PinFunction> {
             _marker: PhantomData<F>,
        }
        
        pub struct Bidirectional<F: PinFunction> {
             _marker: PhantomData<F>,
        }

        // Pin Function states
        pub struct FunctionSio<M: PinMode> {
              _marker: PhantomData<M>,
        }

        pub struct FunctionUart<M: PinMode> {
              _marker: PhantomData<M>,
        }

        pub struct FunctionI2c<M: PinMode> {
              _marker: PhantomData<M>,
        }

        pub struct FunctionPwm<M: PinMode> {
              _marker: PhantomData<M>,
        }

        pub struct NoFunction<M: PinMode> {
              _marker: PhantomData<M>,
        }

        // Pin Mode states
        pub struct PushPull;
        pub struct PullUp;
        pub struct NoMode;
    }

    use typestate::*;


    impl<F: PinFunction> PinDirection for NotConfigured<F> {}
    impl<F: PinFunction> PinDirection for Input<F> {}
    impl<F: PinFunction> PinDirection for Output<F> {}
    impl<F: PinFunction> PinDirection for Bidirectional<F> {}

    impl<M: PinMode> PinFunction for FunctionSio<M> {}
    impl<M: PinMode> PinFunction for FunctionUart<M> {}
    impl<M: PinMode> PinFunction for FunctionI2c<M> {}
    impl<M: PinMode> PinFunction for FunctionPwm<M> {}
    impl<M: PinMode> PinFunction for NoFunction<M> {}

    impl PinMode for PushPull {} 
    impl PinMode for PullUp {} 
    impl PinMode for NoMode {}

    pub struct Pin<D: PinDirection, const INDEX: usize> {
       _marker: PhantomData<D>,
    }

    impl<D: PinDirection, const INDEX: usize> Pin<D, INDEX> {
        fn transition() -> Pin<D, INDEX> { 
            Pin {
                _marker: PhantomData,
            }
        }
    }

    impl<F: PinFunction, const INDEX: usize> Pin<NotConfigured<F>, INDEX> {
        fn new() -> Self { Self { _marker: Default::default() } }
        
        pub fn as_input(&self) -> Pin<Input<F>, INDEX> {        
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*PADS_BANK0::ptr()).gpio(INDEX).modify(|_, w| w.ie().set_bit() );
                }
            });

            Pin::transition()
        }
                
        pub fn as_output(&self) -> Pin<Output<F>, INDEX> {
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*PADS_BANK0::ptr()).gpio(INDEX).modify(|_, w| w.od().clear_bit() );// output enable
                }
            });

            Pin::transition()
        }
        
        pub fn as_bidirectional(&self) -> Pin<Bidirectional<F>, INDEX> {
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*PADS_BANK0::ptr()).gpio(INDEX).modify(|_, w| w
                                                            .od().clear_bit()
                                                            .ie().set_bit() );                }
            });

            Pin::transition()
        }

        pub fn _as_disabled(&self) -> Pin<NotConfigured<F>, INDEX> { unimplemented!() }
    }

    impl<M: PinMode, const INDEX: usize> Pin<Output<NoFunction<M>>, INDEX> {
        pub fn as_sio(&self) -> Pin<Output<FunctionSio<M>>, INDEX> { 
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*IO_BANK0::ptr()).gpio(INDEX).gpio_ctrl().modify(|_, w| w.funcsel().sio());//...as sio 
                    (*SIO::ptr()).gpio_oe().modify(|r, w| w.bits(r.gpio_oe().bits() | 1 << INDEX));// Output enable
                }
            });

            Pin::transition()
        }
        
        pub fn as_uart(&self) -> Pin<Output<FunctionUart<M>>, INDEX> { 
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*IO_BANK0::ptr()).gpio(INDEX).gpio_ctrl().modify(|_, w| w.funcsel().uart());
                }
            });

            Pin::transition()
        }
        
        pub fn as_pwm(&self) -> Pin<Output<FunctionPwm<M>>, INDEX> { 
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*IO_BANK0::ptr()).gpio(INDEX).gpio_ctrl().modify(|_, w| w.funcsel().pwm());//...as pwm 
                }
            });

            Pin::transition()
        }
    }

    impl<M: PinMode, const INDEX: usize> Pin<Bidirectional<NoFunction<M>>, INDEX> {
        pub fn as_i2c(&self) -> Pin<Bidirectional<FunctionI2c<M>>, INDEX> { 
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*IO_BANK0::ptr()).gpio(INDEX).gpio_ctrl().modify(|_, w| w.funcsel().i2c());
                }
            });

            Pin::transition()
        }
    }

    impl<M: PinMode, const INDEX: usize> Pin<Input<NoFunction<M>>, INDEX> {
        pub fn _as_sio(&self) -> Pin<Input<FunctionSio<M>>, INDEX> { 
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*IO_BANK0::ptr()).gpio(INDEX).gpio_ctrl().modify(|_, w| w.funcsel().sio());//...as sio 
                }
            });

            Pin::transition()
        }
        
        pub fn as_uart(&self) -> Pin<Input<FunctionUart<M>>, INDEX> { 
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*IO_BANK0::ptr()).gpio(INDEX).gpio_ctrl().modify(|_, w| w.funcsel().uart());
                }
            });

            Pin::transition()
        }
    }

    impl<const INDEX: usize> Pin<Output<FunctionUart<NoMode>>, INDEX> {
        pub fn as_pushpull(&self) -> Pin<Output<FunctionUart<PushPull>>, INDEX> {
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*PADS_BANK0::ptr()).gpio(INDEX).modify(|_, w| w
                                                        .pue().set_bit()// pull up enable
                                                        .pde().set_bit()// pull down enable
                        );
                }
            });

            Pin::transition()
        }
    }

    impl<const INDEX: usize> Pin<Bidirectional<FunctionI2c<NoMode>>, INDEX> {
        pub fn as_pull_up(&self) -> Pin<Bidirectional<FunctionI2c<PullUp>>, INDEX> {
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*PADS_BANK0::ptr()).gpio(INDEX).modify(|_, w| w
                                                        .pue().set_bit());// pull up enable
                }
            });

            Pin::transition()
        }
    }

    impl<const INDEX: usize> Pin<Input<FunctionUart<NoMode>>, INDEX> {
        pub fn as_pushpull(&self) -> Pin<Input<FunctionUart<PushPull>>, INDEX> {
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*PADS_BANK0::ptr()).gpio(INDEX).modify(|_, w| w
                                                        .pue().set_bit()// pull up enable
                                                        .pde().set_bit());// pull down enable
                }
            });

            Pin::transition()
        }
    }

    impl<const INDEX: usize> Pin<Output<FunctionSio<NoMode>>, INDEX> {
        pub fn as_pushpull(&self) -> Pin<Output<FunctionSio<PushPull>>, INDEX> {
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*PADS_BANK0::ptr()).gpio(INDEX).modify(|_, w| w
                                                        .pue().set_bit()// pull up enable
                                                        .pde().set_bit());// pull down enable
                }
            });

            Pin::transition()
        }
    }

    impl<const INDEX: usize> Pin<Output<FunctionPwm<NoMode>>, INDEX> {
        pub fn as_pushpull(&self) -> Pin<Output<FunctionPwm<PushPull>>, INDEX> {
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*PADS_BANK0::ptr()).gpio(INDEX).modify(|_, w| w
                                                        .pue().set_bit()// pull up enable
                                                        .pde().set_bit());// pull down enable
                }
            });

            Pin::transition()
        }
    }
    
    impl<const INDEX: usize> Pin<Output<FunctionSio<PushPull>>, INDEX> {
        pub fn set_pin(&self) {
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*SIO::ptr()).gpio_out().modify(|r, w| w.bits(r.gpio_out().bits() | 1 << INDEX));// Set pin
                }
            });
        }

        pub fn clear_pin(&self) {
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*SIO::ptr()).gpio_out().modify(|r, w| w.bits(r.gpio_out().bits() & !(1 << INDEX)));// Set pin
                }
            });
        }
        
        pub fn toggle_pin(&self) {
            cortex_m::interrupt::free(|_| {
                unsafe {
                    (*SIO::ptr()).gpio_out().modify(|r, w| w.bits(r.gpio_out().bits() ^ 1 << INDEX));// toggle pin
                }
            });
        }
    }

    pub struct Gpio;

    impl Gpio {
        pub fn new(_: IO_BANK0, _: PADS_BANK0, resets: &Resets) -> Self {
            resets.resets.reset().modify(|_, w| w
                                       .pads_bank0().clear_bit()
                                       .io_bank0().clear_bit());
            Self
        }

        pub fn split(self) -> GpioPins {
            GpioPins::new()
        }
    }

    pub struct GpioPins {
        pub gp0: Pin<NotConfigured<NoFunction<NoMode>>, 0>,
        pub gp1: Pin<NotConfigured<NoFunction<NoMode>>, 1>,
        pub gp4: Pin<NotConfigured<NoFunction<NoMode>>, 4>,
        pub gp5: Pin<NotConfigured<NoFunction<NoMode>>, 5>,
        pub gp8: Pin<NotConfigured<NoFunction<NoMode>>, 8>,
        pub gp9: Pin<NotConfigured<NoFunction<NoMode>>, 9>,
        pub gp16: Pin<NotConfigured<NoFunction<NoMode>>, 16>,
        pub gp17: Pin<NotConfigured<NoFunction<NoMode>>, 17>,
        pub gp20: Pin<NotConfigured<NoFunction<NoMode>>, 20>,
        pub gp25: Pin<NotConfigured<NoFunction<NoMode>>, 25>,
    }

    impl GpioPins {
        fn new() -> Self {
            Self {
                gp0: Pin::new(),
                gp1: Pin::new(),
                gp4: Pin::new(),
                gp5: Pin::new(),
                gp8: Pin::new(),
                gp9: Pin::new(),
                gp16: Pin::new(),
                gp17: Pin::new(),
                gp20: Pin::new(),
                gp25: Pin::new(),
            }
        }
    }
}
